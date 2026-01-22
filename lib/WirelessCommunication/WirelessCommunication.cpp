#include "WirelessCommunication.h"

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role) {
    _nodeAddr = nodeAddr;
    _role = role;

    _anchorTime = millis();
    _lastTxSlotAbs = 0xFFFFFFFF; // Wartosc poczatkowa rozna od 0
    _syncTimeout = 0;

    memset(_lastSeq, 0, sizeof(_lastSeq));

    if (!LoRa.begin(LORA_FREQ)) {
        return false;
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(8);
    LoRa.enableCrc();

    _history.init();

    DBGLN("[API] Begin");

    return true;
}

void WirelessCommunication::poll() {
    readPacket();

    unsigned long now = millis();
    unsigned long elapsed = now - _anchorTime;

    unsigned long absSlot = elapsed / LORA_SLOT;
    unsigned long timeInSlot = elapsed % LORA_SLOT;

    uint8_t currentSlotOwner = (absSlot % MAX_NODES) + 1;
    uint32_t windowStart = LORA_TICK;
    uint32_t windowEnd = LORA_TICK * 2;

    bool isTxWindow = (timeInSlot >= windowStart && timeInSlot < windowEnd);
    bool isMySlot = (currentSlotOwner == _nodeAddr);

    // Warunek transmisji:
    // 1. Jest mój slot
    // 2. Jestem w oknie czasowym (Tick 1)
    // 3. Jeszcze nie wysyłałem w TYM KONKRETNYM slocie absolutnym
    if (isMySlot && isTxWindow && absSlot != _lastTxSlotAbs) {
        WirelessPacket pkt;
        bool shouldSend = false;

        if (_role == ROLE_MASTER && _txQueue.isEmpty()) {
            pkt.type = PKT_REQ;
            pkt.length = 0;
            memset(pkt.trace, 0, MAX_NODES);
            pkt.trace[0] = _nodeAddr;
            pkt.seq = _lastSeq[_nodeAddr-1]++; // Inkrementacja sekwencji SYNC
            shouldSend = true;
        } else if (_role == ROLE_SLAVE && _syncTimeout > now) { // TODO tymczasowe
            pkt.type = PKT_RES;
            pkt.length = 0;
            memset(pkt.trace, 0, MAX_NODES);
            pkt.trace[0] = _nodeAddr;
            pkt.seq = _lastSeq[_nodeAddr-1]++; // Inkrementacja sekwencji SYNC
            shouldSend = true;
        }
        else if (_txQueue.pop(pkt)) {
            shouldSend = true;
        }

        if (shouldSend) {
            uint32_t packetTime = millis();
            writePacket(pkt);
            DBG("[TX] Sent packet ");
            DBG(millis() - packetTime);
            DBGLN(" ms");

            _lastTxSlotAbs = absSlot; // Oznaczamy ten slot absolutny jako obsluzony
        }
    }
}

void WirelessCommunication::syncNetwork(uint8_t senderAddr) {
    unsigned long now = millis();

    unsigned long offset = ((senderAddr - 1) * LORA_TICK) + LORA_TICK + LORA_TOA;

    _anchorTime = now - offset;
    _lastTxSlotAbs = 0xFFFFFFFF;
    _syncTimeout = now + LORA_SLOT * MAX_NODES * 4;
}

bool WirelessCommunication::send(const WirelessPacket& pkt) {
    DBGLN("[API] send() called");
    return _txQueue.push(pkt);
}

bool WirelessCommunication::receive(WirelessPacket& pkt) {
    return _rxQueue.pop(pkt);
}

bool WirelessCommunication::writePacket(WirelessPacket& pkt) {
    WirelessPacketRaw rawPkt;
    encode(pkt, rawPkt);
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&rawPkt, sizeof(WirelessPacketRaw));
    LoRa.endPacket();
    return true;
}

void WirelessCommunication::readPacket() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;
    if (packetSize > sizeof(WirelessPacketRaw)) return;

    WirelessPacketRaw rawPkt;
    WirelessPacket pkt;
    LoRa.readBytes((uint8_t*)&rawPkt, sizeof(WirelessPacketRaw));

    decode(rawPkt, pkt);
    if (pkt.trace[0] == 0 || pkt.trace[0] > MAX_NODES) return;

    handleIncoming(pkt);
}

void WirelessCommunication::handleIncoming(WirelessPacket& pkt) {
    DBGLN("[API] Received packet");

    if (_role == ROLE_SLAVE) {
        if (pkt.type == PKT_REQ){
            DBGLN("[API] Synchronization");
            syncNetwork(pkt.trace[pkt.hopCount - 1]);

            if (pkt.to == _nodeAddr){
                DBGLN("[API] Packet addressed for this node");
                _rxQueue.push(pkt);
                return;
            }
        }
        // mechanizm weryfikacji czy wiadomość była już retransmitowana przez ten węzeł        
        uint32_t now = millis();

        if (_history.contains(pkt.type, pkt.seq, now)) {
            return;
        }
        for (int i = 0; i < MAX_NODES; i++) {
            if (pkt.trace[i] == _nodeAddr) {
                DBGLN("[API] Loop detected dropping packet");
                break;
            }
            if (pkt.trace[i] == 0) {
                DBGLN("[API] Retransmiting packet");
                _history.add(pkt.type, pkt.seq, 3 * LORA_ROUND, now);
                pkt.trace[i] = _nodeAddr;
                _txQueue.push(pkt);
                break;
            }
        }
    }
    
    else if (_role == ROLE_MASTER && pkt.type == PKT_RES) {
        if (pkt.hopCount > 1 || pkt.trace[0] != 0x07){
            DBGLN("[API] Packet addressed for this node");
            _rxQueue.push(pkt);
        }
        else {
            DBGLN("[API] Dropping packet received directly from S1");
        }
    }
}

void WirelessCommunication::encode(const WirelessPacket& logical, WirelessPacketRaw& raw) {
    memset(&raw, 0, sizeof(raw));
    raw.type   = (logical.type & 0xF0) |
                 (logical.to   & 0x0F);
    raw.seq    = logical.seq;
    raw.length = logical.length;
    memcpy(raw.payload, logical.payload, 8);

    for(uint8_t i = 0; i < MAX_NODES / 2; i++) {
        uint8_t from = logical.traceFrom[i] & 0x0F;
        uint8_t to   = logical.traceTo[i]   & 0x0F;

        if(from != 0 || to != 0) {
            raw.trace[i] = (from << 4) | to;
        }
    }
}

void WirelessCommunication::decode(const WirelessPacketRaw& raw, WirelessPacket& logical) {
    memset(&logical, 0, sizeof(logical));
    logical.type   = raw.type & 0xF0;
    logical.to     = raw.type & 0x0F;
    logical.seq    = raw.seq;
    logical.length = raw.length;
    memcpy(logical.payload, raw.payload, 8);

    for(uint8_t i = 0; i < MAX_NODES / 2; i++) {
        uint8_t byte = raw.trace[i];
        if(byte != 0) {
            uint8_t from = (byte >> 4) & 0x0F;
            uint8_t to   =  byte       & 0x0F;

            logical.traceFrom[i] = from;
            logical.traceTo[i]   = to;

            if(from != 0) logical.hopCount++;
            if(to   != 0) logical.hopCount++;
        }
    }
}