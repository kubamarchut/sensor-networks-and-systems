#ifndef BROADCASTSERIAL_H
#define BROADCASTSERIAL_H

#include <Arduino.h>
#include <Crc8.h>
#include <Stopwatch.h>
#include <BroadcastConsts.h>

extern Uart Serial3;

class BroadcastSerial {
private:
    Uart& stream;
    Crc8 rxCrc;
    Stopwatch rxStopwatch = Stopwatch(50);
    int serialIdx;
public:
    bb_buffer rxBuffer = {};

    explicit BroadcastSerial(Uart&, int);

    uint8_t receiveCmd();
    bool receiveData(size_t);
    void reset();

    void begin(unsigned long);
    int available();
    uint8_t read();
    uint8_t readData();
    bool readChecksum();
    void resetRxChecksum();

    void write(uint8_t);
    void flush();
 };

#endif //BROADCASTSERIAL_H