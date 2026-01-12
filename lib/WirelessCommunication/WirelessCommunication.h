
#include <Arduino.h>

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
    PKT_PING      = 2,
    PKT_PONG      = 3,
    PKT_MEASURE   = 4,
    PKT_STATUS    = 5
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

    bool send(uint8_t toAddr, const WirelessPacket& pkt);
    bool hasReceived(WirelessPacket& pkt);

private:
    uint8_t _nodeAddr;
    NodeRole _role;
    uint32_t _slotDurationMs;
    uint32_t _tickDurationMs;

    unsigned long _anchorTime;      // The theoretic start of the whole cycle
    uint8_t _currentSlot;           // 0 to MAX_NODES-1
    uint8_t _currentTick;           // 0 to 4

    // Logic State
    bool _txAllowed;                // Can we TX right now?
    bool _slotHandled;              // Have we already acted in this slot?
    
    // LoRa/Data State
    PacketQueue _txQueue;
    PacketQueue _rxQueue;           // Small buffer for received packets
    uint8_t _lastSeq[MAX_NODES];    // For duplicate detection

     // Internal Functions
    void updateTimeSlot();
    bool sendPacket(WirelessPacket& pkt);
    void receiveLoRa();
    void handleIncoming(WirelessPacket& pkt);
    void syncNetwork(uint8_t senderAddr);
    
    // Queue Helpers
    bool qPush(PacketQueue& q, const WirelessPacket& pkt);
    bool qPop(PacketQueue& q, WirelessPacket& pkt);
    
    // Trace/Dup Helpers
    bool isDuplicate(const WirelessPacket& pkt);
    void updateTrace(WirelessPacket& pkt);

#ifdef LORA_DEBUG
    void dumpPacket(const WirelessPacket& pkt) const;
#endif
};