
#include <Arduino.h>

enum PacketType : uint8_t {
    PACKET_PING = 1,
    PACKET_PONG = 2
};

struct WirelessPacket {
    uint8_t trace[8];//9,2,6,0,0,0k0k0
    uint8_t to;
    uint8_t type;
    uint8_t seq; 
    uint8_t payload[32];
    uint8_t length;
    //uint8_t crc8;
};

class WirelessCommunication {
public:
    bool begin(int nodeAddress);
    bool send(const WirelessPacket& pkt);
    bool receive(WirelessPacket& pkt);

    bool sendPing(uint8_t to);
    bool sendPong(uint8_t to);

private:
    bool receiveLoRa(WirelessPacket& pkt);
    bool sendLoRa(const WirelessPacket& pkt);
};