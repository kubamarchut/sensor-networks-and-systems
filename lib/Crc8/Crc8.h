#ifndef CRC8_H
#define CRC8_H

#include <Arduino.h> 

class Crc8 {
private:
    uint8_t crcValue;
    static const uint8_t POLY = 0x07; 
    static const uint8_t INITIAL_VALUE = 0x00; 

public:
    Crc8();
    void reset();
    void update(uint8_t data);
    uint8_t calculate(const uint8_t* buffer, size_t length);
    uint8_t getCrc() const; 
};

#endif // CRC8_H
