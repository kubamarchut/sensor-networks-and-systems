#ifndef BROADCASTBUS_H
#define BROADCASTBUS_H

#include <Arduino.h>
#include <Crc8.h>

#define BB_MAX_REGISTERS (8)
#define BB_MAX_SENSORS (16)

#define BB_START_BYTE  (0x45)
#define BB_STOP_BYTE   (0x9F)

#define BB_TIMEOUT_MIN (500) //ms
#define BB_TIMEOUT_MAX (1000) //ms


struct bbFrameRegister {
    uint8_t address = 0;
    uint8_t data = 0;
//    uint8_t crc8;
};

struct bbFrameSensor {
    uint8_t address = 0;
    uint8_t registerCount = 0;
    bbFrameRegister registers[BB_MAX_REGISTERS];
//    uint8_t crc8;
};

struct bbFrame {
    uint8_t address = 0;
    uint8_t sequentialNumber = 0;
    uint8_t sensorCount = 0;
    bbFrameSensor sensors[BB_MAX_SENSORS];
//    uint8_t crc8;
};

class BroadcastBus {
private:
    Stream& stream;
    Crc8 crc8;

public:
    bbFrame rxFrame;
    bbFrame txFrame;

    BroadcastBus(Stream&);
    void transmit();

};

#endif // BROADCASTBUS_H