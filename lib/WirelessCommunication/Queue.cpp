//
// Created by deyanix on 17.01.2026.
//

#include "Queue.h"

bool Queue::push(const T& item) {
    if (count >= Size) return false;

    buffer[head] = item;
    head++;
    if (head >= Size) head = 0;

    count++;
    return true;
}

bool Queue::pop(T& item) {
    if (count == 0) return false;

    item = buffer[tail];
    tail++;
    if (tail >= Size) tail = 0;

    count--;
    return true;
}

uint16_t Queue::length() const {
    return count;
}

bool Queue::isFull() const {
    return count >= Size;
}

bool Queue::isEmpty() const {
    return count == 0;
}

void Queue::clear() {
    head = 0;
    tail = 0;
    count = 0;
}