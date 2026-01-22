#include <Arduino.h>
#include "LoRa.h"
#include "Queue.h"
#include "HistoryBuffer.h"
#include "Indicator.h"
#include "Stopwatch.h"

#define MAX_NODES   8
#define QUEUE_SIZE  8
#define LORA_FREQ   868E6
#define LORA_TOA    (185 + 5)
#define LORA_TICK   (LORA_TOA / 3)
#define LORA_SLOT   (LORA_TICK * 5)
#define LORA_ROUND  (LORA_SLOT * MAX_NODES)

//#define LORA_DEBUG
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
    ROLE_SLAVE,
};

enum PacketType : uint8_t {
    PKT_REQ = 0b00000000,
    PKT_RES = 0b10000000,
};

struct __attribute__((packed)) WirelessPacketRaw {
    uint8_t trace[MAX_NODES];
    uint8_t type;
    uint16_t seq;
    uint8_t length;
    uint8_t payload[8];
};

struct __attribute__((packed)) WirelessPacket {
    uint8_t initialTrace[MAX_NODES];
    uint8_t trace[MAX_NODES];
    uint8_t hopCount;
    uint8_t type;
    uint8_t to;
    uint16_t seq;
    uint8_t length;
    uint8_t payload[8];
};

class WirelessCommunication {
public:
    bool begin(uint8_t nodeAddr, NodeRole role, Indicator* indicator);
    void poll();
    bool send(const WirelessPacket& pkt);
    bool receive(WirelessPacket& pkt);
    static void dumpPacket(WirelessPacket& pkt);
    
    private:
    uint8_t _nodeAddr;
    NodeRole _role;
    uint32_t _slotDurationMs;
    
    unsigned long _anchorTime;
    unsigned long _lastTxSlotAbs;
    Indicator* _indicator;
    Stopwatch _rxIndicator;

    Queue<WirelessPacket, QUEUE_SIZE> _txQueue;
    Queue<WirelessPacket, QUEUE_SIZE> _rxQueue;
    HistoryBuffer<QUEUE_SIZE> _history;
    uint8_t _lastSeq[MAX_NODES];
    uint32_t _syncTimeout;
    
    bool writePacket(WirelessPacket& pkt);
    void readPacket();
    void handleIncoming(WirelessPacket& pkt);
    void syncNetwork(uint8_t senderAddr);
    
    static void encode(const WirelessPacket& logical, WirelessPacketRaw& raw);
    static void decode(const WirelessPacketRaw& raw, WirelessPacket& logical);
    static int computeTotalTime(const WirelessPacket& pkt);
};