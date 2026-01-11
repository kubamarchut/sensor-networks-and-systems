
#include <Arduino.h>

#define MAX_NODES       8
#define TX_QUEUE_SIZE   4
#define LORA_DEBUG      1

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
    PKT_PING      = 2,
    PKT_PONG      = 3,
    PKT_MEASURE   = 4,
    PKT_STATUS    = 5
};

struct WirelessPacket {
    uint8_t trace[8];
    uint8_t to;
    uint8_t type;
    uint8_t seq;
    uint8_t payload[32];
    uint8_t length;
};

class WirelessCommunication {
public:
    bool begin(uint8_t nodeAddr, NodeRole role);

    void onSlotStartISR();
    void onGuardEndISR();

    void poll();

    bool send(const WirelessPacket& pkt);
    bool receive(WirelessPacket& pkt);

private:
    uint8_t _nodeAddr;
    NodeRole _role;

    volatile bool _slotFlag = false;
    volatile bool _txAllowed = false;
    volatile uint8_t _slotIndex = 0;

    uint8_t _lastSeq[MAX_NODES] = {0};
    uint8_t _localSeq = 0;

    struct {
        WirelessPacket buffer[TX_QUEUE_SIZE];
        volatile uint8_t head = 0;
        volatile uint8_t tail = 0;
        volatile uint8_t count = 0;
    } _txQueue;

    bool queuePush(const WirelessPacket& pkt);
    bool queuePop(WirelessPacket& pkt);

    bool sendPacket(const WirelessPacket& pkt);
    bool alreadySeen(const WirelessPacket& pkt) const;
    bool appendTrace(WirelessPacket& pkt);
    bool isDuplicate(const WirelessPacket& pkt);

    void handleIncoming(WirelessPacket& pkt);

#ifdef LORA_DEBUG
    void dumpPacket(const WirelessPacket& pkt) const;
#endif
};