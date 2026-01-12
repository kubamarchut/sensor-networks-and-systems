#include "WirelessCommunication.h"

bool WirelessCommunication::begin(uint8_t nodeAddr, NodeRole role, uint32_t slotTimeMs) {
    _nodeAddr = nodeAddr;
    _role = role;
    _slotDurationMs = slotTimeMs;

    _anchorTime = millis();
    _lastTxSlotAbs = 0xFFFFFFFF; // Wartosc poczatkowa rozna od 0

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
    receiveLoRa();

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

        if (_role == ROLE_MASTER && _txQueue.count == 0) {
            pkt.to = 0;
            pkt.type = PKT_TIME_SYNC;
            pkt.length = 0;
            memset(pkt.trace, 0, MAX_NODES);
            pkt.trace[0] = _nodeAddr;
            pkt.seq = _lastSeq[_nodeAddr-1]++; // Inkrementacja sekwencji SYNC
            shouldSend = true;
        }
        else if (qPop(_txQueue, pkt)) {
            shouldSend = true;
        }

        if (shouldSend) {
            DBGLN("[TX] Sending packet");
            sendPacket(pkt);
            _lastTxSlotAbs = absSlot; // Oznaczamy ten slot absolutny jako obsluzony
        }
    }
}

void WirelessCommunication::syncNetwork(uint8_t senderAddr) {
    unsigned long now = millis();
    // Heurystyka czasu lotu + processing
    // TODO ToA!!
    unsigned long heuristicAirTime = 100;

    // Zakładamy, że nadawca wysyła na początku swojego okna (20% slotu - musi matchować windowStart z poll)
    uint32_t expectedTxOffset = _slotDurationMs / 5;

    // senderAddr-1 bo sloty sa 0-indeksowane, adresy 1-indeksowane
    // Obliczamy gdzie teoretycznie powinien zaczac sie caly cykl
    // Offset = (Początek slotu nadawcy) + (Moment nadania wewnątrz slotu) + (Czas lotu)
    unsigned long offset = ((senderAddr - 1) * _slotDurationMs) + expectedTxOffset + heuristicAirTime;

    _anchorTime = now - offset;

    // Reset licznika slotow, zeby nie zablokowac nastepnego cyklu jesli skok czasu byl duzy
    _lastTxSlotAbs = 0xFFFFFFFF;
}

bool WirelessCommunication::send(const WirelessPacket& pkt) {
    DBGLN("[API] send() called");
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
    if (packetSize > sizeof(WirelessPacket)) return;

    WirelessPacket pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(WirelessPacket));

    if (pkt.trace[0] == 0 || pkt.trace[0] > MAX_NODES) return;

    handleIncoming(pkt);
}

void WirelessCommunication::handleIncoming(WirelessPacket& pkt) {
    DBGLN("[API] Received packet");

    if (pkt.type == PKT_TIME_SYNC || pkt.trace[0] == 1) {
        if (_role != ROLE_MASTER) {
            DBGLN("[API] Received TIME_SYNC");
            syncNetwork(pkt.trace[0]);
        }
    }

    if (pkt.type == PKT_DATA) {
        if (pkt.to == _nodeAddr || pkt.to == 0) {
            DBGLN("[API] Received DATA");
            qPush(_rxQueue, pkt);
        }
    }
}

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