#include "CircularBuffer.h"

void CircularBuffer::write(uint8_t data) {
    size_t writeIndex = (index + length) % CIRCULAR_BUFFER_CAPACITY;
    buffer[writeIndex] = data;

    if (length < CIRCULAR_BUFFER_CAPACITY) {
        length++;
    } else {
        index = (index + 1) % CIRCULAR_BUFFER_CAPACITY;
    }
}

uint8_t CircularBuffer::peek() const {
    if (length == 0) {
        return 0x00;
    }
    return buffer[index];
}

uint8_t CircularBuffer::read() {
    if (length == 0) {
        return 0x00;
    }

    uint8_t data = buffer[index];
    index = (index + 1) % CIRCULAR_BUFFER_CAPACITY;
    length--;
    return data;
}

size_t CircularBuffer::available() const {
    return length;
}

void CircularBuffer::clear() {
    index = 0;
    length = 0;
}