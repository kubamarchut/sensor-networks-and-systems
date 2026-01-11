#include "WirelessCommunication.h"
#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 868E6

// ================= INIT =================

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role) {
    _nodeAddr = nodeAddr;
    _role = role;
    memset(_lastSeq, 0, sizeof(_lastSeq));

    DBGLN("[INIT] WirelessCommunication begin");
    DBG("[INIT] Node addr = ");
    DBGLN(_nodeAddr);

    if (!LoRa.begin(LORA_FREQ)) {
        DBGLN("[INIT] LoRa init FAILED");
        return false;
    }

    DBGLN("[INIT] LoRa OK");
    return true;
}

// ================= ISR HOOKS =================

void WirelessCommunication::onSlotStartISR() {
    _slotIndex = (_slotIndex + 1) % MAX_NODES;
    _txAllowed = false;
    _slotFlag = true;

#ifdef LORA_DEBUG
    DBG("[TDMA] Slot start → ");
    DBGLN(_slotIndex + 1);
#endif
}

void WirelessCommunication::onGuardEndISR() {
    _txAllowed = true;
    DBGLN("[TDMA] Guard ended – TX allowed");
}

// ================= MAIN LOOP =================

void WirelessCommunication::poll() {
    if (_slotFlag && _txAllowed && (_slotIndex + 1 == _nodeAddr)) {
        _slotFlag = false;

        WirelessPacket pkt;
        if (queuePop(pkt)) {
            DBGLN("[TX] Slot TX");
#ifdef LORA_DEBUG
            dumpPacket(pkt);
#endif
            sendPacket(pkt);
            _txAllowed = false;
        } else {
            DBGLN("[TX] Slot but queue empty");
        }
    }
}

// ================= API =================

bool WirelessCommunication::send(const WirelessPacket& pkt) {
    DBGLN("[API] send() called");
    return queuePush(pkt);
}

// ================= RECEIVE =================

bool WirelessCommunication::receive(WirelessPacket& pkt) {
    int size = LoRa.parsePacket();
    if (size <= 0) return false;

    DBGLN("[RX] Packet received");

    LoRa.readBytes((uint8_t*)&pkt, sizeof(WirelessPacket));

#ifdef LORA_DEBUG
    dumpPacket(pkt);
#endif

    handleIncoming(pkt);
    return true;
}

void WirelessCommunication::handleIncoming(WirelessPacket& pkt) {
    if (alreadySeen(pkt)) {
        DBGLN("[DROP] Loop detected");
        return;
    }

    if (isDuplicate(pkt)) {
        DBGLN("[DROP] Duplicate packet");
        return;
    }

    appendTrace(pkt);

    if (pkt.type == PKT_TIME_SYNC) {
        DBGLN("[SYNC] Time sync decoded");
    }

    if (_role == ROLE_RELAY) {
        DBGLN("[RELAY] Enqueue for retransmission");
        queuePush(pkt);
    }
}

// ================= TRACE / DUP =================

bool WirelessCommunication::alreadySeen(const WirelessPacket& pkt) const {
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (pkt.trace[i] == 0) break;
        if (pkt.trace[i] == _nodeAddr) return true;
    }
    return false;
}

bool WirelessCommunication::appendTrace(WirelessPacket& pkt) {
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (pkt.trace[i] == 0) {
            pkt.trace[i] = _nodeAddr;
            return true;
        }
    }
    DBGLN("[DROP] Trace full");
    return false;
}

bool WirelessCommunication::isDuplicate(const WirelessPacket& pkt) {
    uint8_t origin = pkt.trace[0];
    if (origin < 1 || origin > MAX_NODES) return true;

    uint8_t idx = origin - 1;
    if (pkt.seq == _lastSeq[idx]) return true;

    _lastSeq[idx] = pkt.seq;
    return false;
}

// ================= QUEUE =================

bool WirelessCommunication::queuePush(const WirelessPacket& pkt) {
    if (_txQueue.count >= TX_QUEUE_SIZE) {
        DBGLN("[QUEUE] FULL - drop");
        return false;
    }

    _txQueue.buffer[_txQueue.head] = pkt;
    _txQueue.head = (_txQueue.head + 1) % TX_QUEUE_SIZE;
    _txQueue.count++;

    DBG("[QUEUE] PUSH count=");
    DBGLN(_txQueue.count);
    return true;
}

bool WirelessCommunication::queuePop(WirelessPacket& pkt) {
    if (_txQueue.count == 0) return false;

    pkt = _txQueue.buffer[_txQueue.tail];
    _txQueue.tail = (_txQueue.tail + 1) % TX_QUEUE_SIZE;
    _txQueue.count--;

    DBG("[QUEUE] POP count=");
    DBGLN(_txQueue.count);
    return true;
}

// ================= RADIO =================

bool WirelessCommunication::sendPacket(const WirelessPacket& pkt) {
    DBGLN("[RADIO] TX begin");
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(WirelessPacket));
    bool ok = LoRa.endPacket();
    DBGLN(ok ? "[RADIO] TX done" : "[RADIO] TX failed");
    return ok;
}

// ================= DEBUG =================

#ifdef LORA_DEBUG
void WirelessCommunication::dumpPacket(const WirelessPacket& pkt) const {
    Serial.print("[PKT] to=");
    Serial.print(pkt.to);
    Serial.print(" type=");
    Serial.print(pkt.type);
    Serial.print(" seq=");
    Serial.print(pkt.seq);
    Serial.print(" len=");
    Serial.print(pkt.length);
    Serial.print(" trace=");

    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (pkt.trace[i] == 0) break;
        Serial.print(pkt.trace[i]);
        Serial.print(" ");
    }
    Serial.println();
}
#endif
