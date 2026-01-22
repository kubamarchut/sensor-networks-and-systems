#include "WirelessCommunication.h"

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role, Indicator* indicator) {
    _nodeAddr = nodeAddr;
    _role = role;
    _indicator = indicator;

    _anchorTime = millis();
    _lastTxSlotAbs = 0xFFFFFFFF; // Wartosc poczatkowa rozna od 0
    _syncTimeout = 0;
    _rxIndicator = Stopwatch(0xFFFFFFFF);

    memset(_lastSeq, 0, sizeof(_lastSeq));

    if (!LoRa.begin(LORA_FREQ)) {
        return false;
    }

    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(8);
    LoRa.setTxPower(20);
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

        if (_txQueue.pop(pkt)) {
            uint32_t packetTime = millis();
            writePacket(pkt);
            DBG("[TX] Sent packet ");
            DBG(millis() - packetTime);
            DBGLN(" ms");

            _lastTxSlotAbs = absSlot; // Oznaczamy ten slot absolutny jako obsluzony
        }
    }

    
    if (_rxIndicator.isTimeout()) {
        _indicator->blue(false);
        _rxIndicator.reset(0xFFFFFFFF);
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
    _indicator->red(true);
    WirelessPacketRaw rawPkt;
    encode(pkt, rawPkt);
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&rawPkt, sizeof(WirelessPacketRaw));
    LoRa.endPacket();
    _indicator->red(false);

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

    _indicator->blue(true);
    _rxIndicator.reset(LORA_TOA);
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

    for(uint8_t i = 0; i < MAX_NODES; i++) {
        uint8_t from = logical.initialTrace[i] & 0x0F;
        uint8_t to   = logical.trace[i]        & 0x0F;

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

    for(uint8_t i = 0; i < MAX_NODES; i++) {
        uint8_t byte = raw.trace[i];
        if(byte != 0) {
            uint8_t from = (byte >> 4) & 0x0F;
            uint8_t to   =  byte       & 0x0F;

            logical.initialTrace[i] = from;
            logical.trace[i]        = to;

            //if(from != 0) logical.hopCount++;
            if(to   != 0) logical.hopCount++;
        }
    }
}

void WirelessCommunication::dumpPacket(WirelessPacket& pkt){
    Serial.print(F("PKT "));

    // Inter-transmission time (example: using seq)
    Serial.print(F("Itc="));
    Serial.print(WirelessCommunication::computeTotalTime(pkt));
    Serial.print(F("ms "));

    // Payload
    Serial.print(F("data=("));
    for (uint8_t i = 0; i < pkt.length && i < sizeof(pkt.payload); i++) {
        Serial.print(pkt.payload[i]);
        if (i + 1 < pkt.length)
            Serial.print(F(", "));
    }
    Serial.print(F(") "));

    // Trace
    Serial.print(F("S"));
    Serial.print(pkt.initialTrace[0]);

    for (uint8_t i = 1; i < pkt.hopCount && i < MAX_NODES; i++) {
        Serial.print(F("->W"));
        Serial.print(pkt.trace[i]-1);
    }

    Serial.print(F("->W0"));
    Serial.println();
}

int WirelessCommunication::computeTotalTime(const WirelessPacket& pkt)
{
    if (pkt.hopCount == 0) return 0;

    int totalSlots = 1;

    for (size_t i = 1; i < pkt.hopCount; i++) {
        if (pkt.trace[i - 1] <= pkt.trace[i]) {
            totalSlots += pkt.trace[i] - pkt.trace[i - 1];
        } else {
            totalSlots += MAX_NODES - pkt.trace[i - 1] + pkt.trace[i];
        }
    }

    return totalSlots * LORA_SLOT - 2 * LORA_TICK;
}