#include "WirelessCommunication.h"

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role, uint32_t slotTimeMs) {
    _nodeAddr = nodeAddr;
    _role = role;
    _slotDurationMs = slotTimeMs;

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
    LoRa.enableCrc();

    _history.init();

    DBGLN("[API] Begin");

    return true;
}

void WirelessCommunication::poll() {
    readPacket();

    unsigned long now = millis();
    unsigned long elapsed = now - _anchorTime;

    // Obliczenie absolutnego numeru slotu od startu
    unsigned long absSlot = elapsed / _slotDurationMs;

    // Obliczenie offsetu wewnatrz slotu
    unsigned long timeInSlot = elapsed % _slotDurationMs;

    // Ustalenie czyj to slot (zakladamy adresy 1..MAX_NODES, sloty 0..MAX-1)
    // Cykl powtarza sie co MAX_NODES
    uint8_t currentSlotOwner = (absSlot % MAX_NODES) + 1;

    // Definicja okna transmisji: np. 20% - 40% czasu trwania slotu (Tick 1)
    uint32_t windowStart = _slotDurationMs / 5;
    uint32_t windowEnd = windowStart * 2;

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
    // Heurystyka czasu lotu + processing
    // TODO ToA!!
    unsigned long heuristicAirTime = 170;

    // Zakładamy, że nadawca wysyła na początku swojego okna (20% slotu - musi matchować windowStart z poll)
    uint32_t expectedTxOffset = _slotDurationMs / 5;

    // senderAddr-1 bo sloty sa 0-indeksowane, adresy 1-indeksowane
    // Obliczamy gdzie teoretycznie powinien zaczac sie caly cykl
    // Offset = (Początek slotu nadawcy) + (Moment nadania wewnątrz slotu) + (Czas lotu)
    unsigned long offset = ((senderAddr - 1) * _slotDurationMs) + expectedTxOffset + heuristicAirTime;

    _anchorTime = now - offset;

    // Reset licznika slotow, zeby nie zablokowac nastepnego cyklu jesli skok czasu byl duzy
    _lastTxSlotAbs = 0xFFFFFFFF;
    _syncTimeout = now + _slotDurationMs * MAX_NODES * 4;
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
                _history.add(pkt.type, pkt.seq, 3 * _slotDurationMs, now);
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
        uint8_t high = logical.trace[2 * i]     & 0x0F;
        uint8_t low  = logical.trace[2 * i + 1] & 0x0F;

        if(high != 0 || low != 0) {
            raw.trace[i] = (high << 4) | low;
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
            uint8_t high = (byte >> 4) & 0x0F;
            uint8_t low  =  byte       & 0x0F;

            logical.trace[2 * i]     = high;
            logical.trace[2 * i + 1] = low;

            if(high != 0) logical.hopCount++;
            if(low  != 0) logical.hopCount++;
        }
    }
}