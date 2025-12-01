#ifndef BROADCASTBUS_H
#define BROADCASTBUS_H

#include <Arduino.h>
#include <Crc8.h>
#include <BroadcastSerial.h>

#define BB_MAX_REGISTERS (8)
#define BB_MAX_SENSORS (88)

#define BB_MASK_START  (0x90)
#define BB_MASK_STOP   (0xA0)
#define BB_MASK_REQ    (0x03)
#define BB_MASK_SEN    (0x0A)
#define BB_MASK_FIN    (0x0D)

#define BB_BAUD (9600)
#define BB_RECV_TIMEOUT (400) // ms
#define BB_ECHO_TIMEOUT (50) // ms
#define BB_DELAY_MIN (500) //ms
#define BB_DELAY_MAX (1000) //ms

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t data;
} bb_sensor_register;

typedef struct __attribute__((packed)) {
    uint8_t node_addr;
    uint8_t sensor_addr;
    uint8_t seq;
    uint8_t flags;
    uint8_t regs_len;
    bb_sensor_register regs[BB_MAX_REGISTERS];
} bb_sensor_frame;

class BroadcastBus {
private:
    uint8_t txCmd = 0;
    Crc8 txCrc;
    Crc8 rxCrc;
public:
    void begin();

    void startTransmission(uint8_t);
    void endTransmission();
    void sendRequest(uint8_t);
    void sendSensor(bb_sensor_frame&);
    void sendFinish(uint8_t);

    void write(uint8_t);
    void writeData(uint8_t);
    void writeData(uint8_t*, size_t);
    void flush();
};


#endif // BROADCASTBUS_H