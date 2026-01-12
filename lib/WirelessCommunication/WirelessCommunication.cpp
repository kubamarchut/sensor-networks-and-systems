#include "WirelessCommunication.h"
#include <SPI.h>
#include <LoRa.h>
#include "WirelessCommunication.h"

// ================= INIT =================

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role, uint32_t slotTimeMs) {
    _nodeAddr = nodeAddr;
    _role = role;
    _slotDurationMs = slotTimeMs;
    _tickDurationMs = slotTimeMs / 5; // 5 Ticks per slot
    
    // Reset State
    _anchorTime = millis();
    _currentSlot = 0;
    _currentTick = 0;
    _txAllowed = false;
    _slotHandled = false;
    
    memset(_lastSeq, 0, sizeof(_lastSeq));

    Serial.print("[INIT] Node "); Serial.print(_nodeAddr);
    Serial.print(" SlotTime: "); Serial.println(_slotDurationMs);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[INIT] LoRa Failed!");
        return false;
    }

    LoRa.setSpreadingFactor(12); // SF12 for range, slow speed
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
    
    // IMPORTANT: Enable CRC to ensure we don't process garbage
    LoRa.enableCrc();

    Serial.println("[INIT] Ready");
    return true;
}

// ================= MAIN LOOP =================

void WirelessCommunication::poll() {
    // 1. Check for incoming LoRa packets (Always listening)
    receiveLoRa();

    // 2. Update Time Logic (Slots & Ticks)
    updateTimeSlot();

    // 3. Handle Transmission (Only if conditions met)
    // Condition: My Slot AND Tick 1 AND haven't sent yet
    if (_txAllowed && (_currentSlot + 1 == _nodeAddr) && !_slotHandled) {
        
        WirelessPacket pkt;
        
        // If we are Master, we prioritize sending a SYNC packet if queue is empty
        // Or we just attach Sync flag to data.
        // For simplicity: If Master queue is empty, send explicit SYNC.
        if (_role == ROLE_MASTER && _txQueue.count == 0) {
             pkt.to = 0; // Broadcast
             pkt.type = PKT_TIME_SYNC;
             pkt.length = 0;
             memset(pkt.trace, 0, MAX_NODES);
             pkt.trace[0] = _nodeAddr;
             // Push to temp variable, not queue
             sendPacket(pkt);
             _slotHandled = true;
        }
        else if (qPop(_txQueue, pkt)) {
            // Normal Data Send
            Serial.println("[TDMA] Tick 1: Sending Data");
            sendPacket(pkt);
            _slotHandled = true;
        }
    }
}

// ================= TIMING LOGIC =================

void WirelessCommunication::updateTimeSlot() {
    unsigned long now = millis();
    unsigned long timeSinceAnchor = now - _anchorTime;
    unsigned long cycleDuration = MAX_NODES * _slotDurationMs;

    // Calculate position in cycle
    unsigned long timeInCycle = timeSinceAnchor % cycleDuration;
    
    uint8_t newSlot = timeInCycle / _slotDurationMs;
    uint8_t newTick = (timeInCycle % _slotDurationMs) / _tickDurationMs;

    // Detect State Change
    if (newSlot != _currentSlot || newTick != _currentTick) {
        _currentSlot = newSlot;
        _currentTick = newTick;

        // Reset slot handled flag if we moved to a new slot
        if (newTick == 0) {
            _slotHandled = false;
        }

        // --- TICK LOGIC ---
        // TICK 1 is the TX Window (2nd mini-slot)
        if (_currentTick == 1) {
            _txAllowed = true;
        } else {
            _txAllowed = false;
        }
    }
}

// ================= SYNC LOGIC =================

void WirelessCommunication::syncNetwork(uint8_t senderAddr) {
    // SIMPLE SYNC:
    // If we hear from Node X, we assume the current time is exactly
    // the end of Node X's slot.
    // So we reset _anchorTime so that (Now - Anchor) matches that logic.

    unsigned long now = millis();
    
    // Example: If we hear Node 1 (Sender 1).
    // The cycle should have started roughly (1 * SlotDuration) ago.
    // We add a tiny offset for transmission time (e.g. 50ms) purely heuristic.
    unsigned long heuristicAirTime = 100; 

    // Calculate where the Anchor SHOULD be based on who just spoke
    unsigned long offset = (senderAddr * _slotDurationMs) + heuristicAirTime;
    
    _anchorTime = now - offset;

    Serial.print("[SYNC] Resync from Node ");
    Serial.println(senderAddr);
}

// ================= TX / RX API =================

bool WirelessCommunication::send(uint8_t toAddr, const char* data) {
    WirelessPacket pkt;
    pkt.to = toAddr;
    pkt.type = PKT_DATA;
    pkt.seq = _lastSeq[_nodeAddr-1] + 1; // Increment my seq
    _lastSeq[_nodeAddr-1]++;
    
    // Copy payload
    pkt.length = strlen(data);
    if (pkt.length > 32) pkt.length = 32;
    memcpy(pkt.payload, data, pkt.length);

    // Trace setup
    memset(pkt.trace, 0, MAX_NODES);
    pkt.trace[0] = _nodeAddr;

    return qPush(_txQueue, pkt);
}

bool WirelessCommunication::hasReceived(WirelessPacket& pkt) {
    return qPop(_rxQueue, pkt);
}

bool WirelessCommunication::sendPacket(WirelessPacket& pkt) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(WirelessPacket));
    LoRa.endPacket();
    return true;
}

void WirelessCommunication::receiveLoRa() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    WirelessPacket pkt;
    if (packetSize > sizeof(WirelessPacket)) return; // Too big/garbage

    LoRa.readBytes((uint8_t*)&pkt, sizeof(WirelessPacket));

    // Filter Bad Packets
    if (pkt.trace[0] == 0 || pkt.trace[0] > MAX_NODES) return;

    handleIncoming(pkt);
}

void WirelessCommunication::handleIncoming(WirelessPacket& pkt) {
    // 1. SYNC Check
    // If it's a SYNC packet, OR if it's from Master (Node 1), we sync.
    if (pkt.type == PKT_TIME_SYNC || pkt.trace[0] == 1) {
        // Only Slaves sync to Master. Master never syncs to Slaves.
        if (_role == ROLE_SLAVE) {
            syncNetwork(pkt.trace[0]);
        }
    }

    // 2. Data processing
    if (pkt.type == PKT_DATA) {
        if (pkt.to == _nodeAddr || pkt.to == 0) {
             // It's for us!
             qPush(_rxQueue, pkt);
        }
    }
}

// ================= UTILS =================

bool WirelessCommunication::qPush(PacketQueue& q, const WirelessPacket& pkt) {
    if (q.count >= TX_QUEUE_SIZE) return false;
    q.buffer[q.head] = pkt;
    q.head = (q.head + 1) % TX_QUEUE_SIZE;
    q.count++;
    return true;
}

bool WirelessCommunication::qPop(PacketQueue& q, WirelessPacket& pkt) {
    if (q.count == 0) return false;
    pkt = q.buffer[q.tail];
    q.tail = (q.tail + 1) % TX_QUEUE_SIZE;
    q.count--;
    return true;
}