#ifndef HISTORY_BUFFER_H
#define HISTORY_BUFFER_H

#include <stdint.h>
#include <stddef.h>

struct HistoryEntry {
    uint8_t  type;
    uint8_t  seq;
    uint32_t expiresAt;
    bool     valid;
};

template <size_t SIZE>
class HistoryBuffer {
public:
    void init() {
        for (size_t i = 0; i < SIZE; i++) {
            entries[i].valid = false;
        }
    }

    void cleanup(uint32_t now) {
        for (size_t i = 0; i < SIZE; i++) {
            if (entries[i].valid && timeExpired(entries[i].expiresAt, now)) {
                entries[i].valid = false;
            }
        }
    }

    bool contains(uint8_t type, uint8_t seq, uint32_t now) {
        cleanup(now);

        for (size_t i = 0; i < SIZE; i++) {
            if (entries[i].valid &&
                entries[i].type == type &&
                entries[i].seq  == seq) {
                return true;
            }
        }
        return false;
    }

    void add(uint8_t type, uint8_t seq, uint32_t ttl, uint32_t now) {
        cleanup(now);

        for (size_t i = 0; i < SIZE; i++) {
            if (!entries[i].valid) {
                writeEntry(i, type, seq, ttl, now);
                return;
            }
        }

        size_t oldest = findOldest();
        writeEntry(oldest, type, seq, ttl, now);
    }

private:
    HistoryEntry entries[SIZE];

    void writeEntry(size_t i, uint8_t type, uint8_t seq,
                    uint32_t ttl, uint32_t now) {
        entries[i].type      = type;
        entries[i].seq       = seq;
        entries[i].expiresAt = now + ttl;
        entries[i].valid     = true;
    }

    size_t findOldest() {
        size_t oldest = 0;
        uint32_t minTime = entries[0].expiresAt;

        for (size_t i = 1; i < SIZE; i++) {
            if (entries[i].expiresAt < minTime) {
                minTime = entries[i].expiresAt;
                oldest = i;
            }
        }
        return oldest;
    }

    bool timeExpired(uint32_t expiry, uint32_t now) {
        return (int32_t)(now - expiry) >= 0;
    }
};

#endif
