#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <Arduino.h>

class Stopwatch {
public:
    explicit Stopwatch(uint32_t timeout = 0);

    void reset();
    void reset(uint32_t newTimeout);
    void setTimeout(uint32_t newTimeout);
    uint32_t getElapsed() const;
    bool isTimeout() const;

private:
    uint32_t startTime;
    uint32_t timeout;
};

#endif //STOPWATCH_H