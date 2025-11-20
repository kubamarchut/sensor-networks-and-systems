#include "BroadcastBus.h"

BroadcastBus::BroadcastBus(arduino::Stream& stream) : stream(stream) {
}

void BroadcastBus::transmit() {
    uint8_t pointer = random();

    stream.write(BB_START_BYTE);
    stream.write(pointer);


}