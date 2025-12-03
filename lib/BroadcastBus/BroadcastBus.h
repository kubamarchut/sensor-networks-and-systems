#ifndef BROADCASTBUS_H
#define BROADCASTBUS_H

#include <Arduino.h>
#include <Crc8.h>
#include <BroadcastConsts.h>
#include <BroadcastSerial.h>

class BroadcastBus {
private:
    uint8_t txCmd = 0;
    Crc8 txCrc;
public:
    BroadcastSerial bSerial1 = BroadcastSerial(Serial1);
    BroadcastSerial bSerial2 = BroadcastSerial(Serial3);

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