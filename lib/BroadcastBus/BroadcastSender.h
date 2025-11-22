#ifndef BROADCASTSENDER_H
#define BROADCASTSENDER_H

#include "BroadcastBus.h"
#include "CircularBuffer.h"
#include <Crc8.h>
#include <Stopwatch.h>

class BroadcastSender {
public:
    bbFrame frame;
    bool frameComplete = false;

    BroadcastSender(Stream&, uint8_t);
    void transmit();
private:
    Stream& stream;
    Crc8 crc8;
    CircularBuffer buffer;
    Stopwatch stopwatch;

    uint8_t address;
    uint8_t seq = 0;
    size_t currentSensor = 0;
    bbState state = IDLE;

    void write(uint8_t);
    void suspend();
};

#endif // BROADCASTSENDER_H