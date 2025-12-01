#ifndef BROADCASTSERIAL_H
#define BROADCASTSERIAL_H

#include <Arduino.h>

extern Uart Serial3;

void Serial3_init();

// class BroadcastSerial {
// private:
//     Stream& stream;
//     Crc8 txCrc;
//     Crc8 rxCrc;
// public:
//     explicit BroadcastSerial(Stream&);
//     void write(uint8_t);
//     void writeData(uint8_t);
//     void writeData(uint8_t*, size_t);
//     void flush();
// };

#endif //BROADCASTSERIAL_H