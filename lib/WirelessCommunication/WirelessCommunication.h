#include <Arduino.h>
#include "LoRa.h"

#define MAX_NODES       8       
#define TX_QUEUE_SIZE   8       
#define LORA_FREQ       868E6

#ifdef LORA_DEBUG
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
  #define DBGHEX(x) Serial.print(x, HEX)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGHEX(x)
#endif

enum NodeRole : uint8_t {
    ROLE_MASTER,
    ROLE_RELAY,
    ROLE_LEAF
};

enum PacketType : uint8_t {
    PKT_TIME_SYNC = 1,
    PKT_DATA      = 2, // Poprawiono nazewnictwo zg z impl
    PKT_PING      = 3, // Opcjonalne
    PKT_PONG      = 4  // Opcjonalne
};

struct WirelessPacket {
    uint8_t trace[MAX_NODES];
    uint8_t to;
    uint8_t type;
    uint8_t seq;
    uint8_t payload[32];
    uint8_t length;
};

struct PacketQueue {
    WirelessPacket buffer[TX_QUEUE_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
};

class WirelessCommunication {
public:
    bool begin(uint8_t nodeAddr, NodeRole role, uint32_t slotTimeMs);
    void poll();
    bool send(const WirelessPacket& pkt);
    bool hasReceived(WirelessPacket& pkt);

private:
    uint8_t _nodeAddr;
    NodeRole _role;
    uint32_t _slotDurationMs;

    unsigned long _anchorTime;
    unsigned long _lastTxSlotAbs;   // Zamiast flag bool - absolutny numer obsłużonego slotu

    PacketQueue _txQueue;
    PacketQueue _rxQueue;
    uint8_t _lastSeq[MAX_NODES];

    bool sendPacket(WirelessPacket& pkt);
    void receiveLoRa();
    void handleIncoming(WirelessPacket& pkt);
    void syncNetwork(uint8_t senderAddr);

    bool qPush(PacketQueue& q, const WirelessPacket& pkt);
    bool qPop(PacketQueue& q, WirelessPacket& pkt);

#ifdef LORA_DEBUG
    void dumpPacket(const WirelessPacket& pkt) const;
#endif
};