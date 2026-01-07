#include "WirelessCommunication.h"

#define USE_LORA

#ifdef USE_LORA
#include <SPI.h>
#include <LoRa.h>
#define LORA_FREQ 8681E5
#endif

bool WirelessCommunication::begin(int nodeAddr) {
#ifdef USE_LORA
    return LoRa.begin(LORA_FREQ);
#else
    return false;
#endif
}

bool WirelessCommunication::send(const WirelessPacket& pkt) {
#ifdef USE_LORA
    return sendLoRa(pkt);
#else
    return false;
#endif
}

bool WirelessCommunication::receive(WirelessPacket& pkt) {
#ifdef USE_LORA
    return receiveLoRa(pkt);
#else
    return false;
#endif
}

bool WirelessCommunication::sendPing(uint8_t to) {
    WirelessPacket pkt;
    pkt.from   = from;
    pkt.to     = to;
    pkt.type   = PACKET_PING;
    pkt.length = 0;
    return send(pkt);
}

bool WirelessCommunication::sendPong(uint8_t to) {
    WirelessPacket pkt;
    pkt.from   = from;
    pkt.to     = to;
    pkt.type   = PACKET_PONG;
    pkt.length = 0;
    return send(pkt);
}

bool WirelessCommunication::sendLoRa(const WirelessPacket& pkt) {
    LoRa.beginPacket();
    LoRa.write(pkt.from);
    LoRa.write(pkt.to);
    LoRa.write(pkt.type);
    LoRa.write(pkt.length);
    LoRa.write(pkt.payload, pkt.length);
    //LoRa.write(pkt.crc8);
    return LoRa.endPacket();
}

bool WirelessCommunication::receiveLoRa(WirelessPacket& pkt) {
    int size = LoRa.parsePacket();
    if (size < 4) return false;

    pkt.from   = LoRa.read();
    pkt.to     = LoRa.read();
    pkt.type   = LoRa.read();
    pkt.length = LoRa.read();

    if (pkt.length > sizeof(pkt.payload)) return false;

    LoRa.readBytes(pkt.payload, pkt.length);
    return true;
}
