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
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
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
            pkt.to = 0;
            pkt.type = PKT_REQ;
            pkt.length = 0;
            memset(pkt.trace, 0, MAX_NODES);
            pkt.trace[0] = _nodeAddr;
            pkt.seq = _lastSeq[_nodeAddr-1]++; // Inkrementacja sekwencji SYNC
            shouldSend = true;
        } else if (_role == ROLE_SLAVE && _syncTimeout > now) { // TODO tymczasowe
            pkt.to = 0;
            pkt.type = PKT_DATA;
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
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pkt, sizeof(WirelessPacket));
    LoRa.endPacket();
    return true;
}

void WirelessCommunication::readPacket() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;
    if (packetSize > sizeof(WirelessPacket)) return;

    WirelessPacket pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(WirelessPacket));

    if (pkt.trace[0] == 0 || pkt.trace[0] > MAX_NODES) return;

    handleIncoming(pkt);
}

void WirelessCommunication::handleIncoming(WirelessPacket& pkt) {
    DBGLN("[API] Received packet");

    if (pkt.type == PKT_REQ && _role == ROLE_SLAVE) {
        DBGLN("[API] Received TIME_SYNC by slave");
        syncNetwork(pkt.trace[0]);
    }

    if (pkt.type == PKT_RES) {
        if (_role == ROLE_MASTER) {
            DBGLN("[API] Received DATA by master");
            _rxQueue.push(pkt);
        } else if (_role == ROLE_SLAVE) {
            DBGLN("[API] Received DATA by slave");
            for (int i = 0; i < MAX_NODES; i++) {
                if (!pkt.trace[i]) {
                    pkt.trace[i] = _nodeAddr;
                    _txQueue.push(pkt);
                    break;
                }
            }
        }
    }
}