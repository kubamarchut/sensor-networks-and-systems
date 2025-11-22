#ifndef BROADCASTBUS_H
#define BROADCASTBUS_H

#include <Arduino.h>

#define BB_MAX_REGISTERS (8)
#define BB_MAX_SENSORS (16)

#define BB_START_BYTE  (0x45)
#define BB_STOP_BYTE   (0x9F)

#define BB_RECV_TIMEOUT (400) // ms
#define BB_ECHO_TIMEOUT (50) // ms
#define BB_DELAY_MIN (500) //ms
#define BB_DELAY_MAX (1000) //ms

struct bbFrameRegister {
    uint8_t address = 0;
    uint8_t data = 0;
};

struct bbFrameSensor {
    uint8_t address = 0;
    uint8_t registerCount = 0;
    bbFrameRegister registers[BB_MAX_REGISTERS];
};

struct bbFrame {
    uint8_t address = 0;
    uint8_t sequentialNumber = 0;
    uint8_t sensorCount = 0;
    bbFrameSensor sensors[BB_MAX_SENSORS];
};

enum bbState {
    SUSPENDED = -1,
    IDLE = 0,
    PREAMBLE = 1,
    FRAME_METADATA = 2,
    SENSOR_METADATA = 3,
    REGISTER = 4,
    TERMINATOR = 5,
};

#endif // BROADCASTBUS_H