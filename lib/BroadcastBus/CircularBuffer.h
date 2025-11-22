#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

#define CIRCULAR_BUFFER_CAPACITY (128)

class CircularBuffer {
private:
    uint8_t buffer[CIRCULAR_BUFFER_CAPACITY];
    size_t index = 0;
    size_t length = 0;
public:
    void write(uint8_t data);
    uint8_t peek() const;
    uint8_t read();
    size_t available() const;
    void clear();
};

#endif