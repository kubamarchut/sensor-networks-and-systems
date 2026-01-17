#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>

template <typename T, uint16_t Size>
class Queue {
public:
    Queue() : head(0), tail(0), count(0) {}

    bool push(const T& item) {
        if (count >= Size) return false;

        buffer[head] = item;
        head++;
        if (head >= Size) head = 0;

        count++;
        return true;
    }

    bool pop(T& item) {
        if (count == 0) return false;

        item = buffer[tail];
        tail++;
        if (tail >= Size) tail = 0;

        count--;
        return true;
    }

    uint16_t length() const {
        return count;
    }

    bool isFull() const {
        return count >= Size;
    }

    bool isEmpty() const {
        return count == 0;
    }

    void clear() {
        head = 0;
        tail = 0;
        count = 0;
    }

private:
    T buffer[Size];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
};


#endif //QUEUE_H