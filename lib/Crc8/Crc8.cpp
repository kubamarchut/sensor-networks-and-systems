#include "Crc8.h"

Crc8::Crc8() { 
    reset(); 
}

void Crc8::reset() { 
    crcValue = INITIAL_VALUE; 
}

void Crc8::update(uint8_t data) {
    crcValue ^= data; 

    for (int i = 0; i < 8; i++) {
        if (crcValue & 0x80) {
            crcValue = (crcValue << 1) ^ POLY; 
        } else {
            crcValue <<= 1;
        }
    }
}

uint8_t Crc8::calculate(const uint8_t* buffer, size_t length) {
    reset();
    for (size_t i = 0; i < length; i++) {
        update(buffer[i]);
    }
    return crcValue;
}

uint8_t Crc8::getCrc() const { 
    return crcValue; 
}
