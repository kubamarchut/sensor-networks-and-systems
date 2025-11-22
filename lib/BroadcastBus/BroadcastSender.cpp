#include "BroadcastSender.h"

BroadcastSender::BroadcastSender(Stream& stream, uint8_t address) : stream(stream), address(address) {
}

void BroadcastSender::transmit() {
    // 1. Sprawdzamy dane na strumieniu wejściowym
    if (stream.available()) {
        if (buffer.available()) {
            // Weryfikacja zabuforowanych wysłanych danych
            while (stream.available()) {
                uint8_t data = stream.read();
                if (buffer.peek() == data) {
                    buffer.read();
                }
            }
        } else {
            // Wykryto dane na wyjściu w czasie, gdy nie było nic wysyłane
            suspend();
            return;
        }
    }

    // 2. Obsługa stanu zawieszenia
    if (state == SUSPENDED) {
        if (stopwatch.isTimeout()) {
            state = IDLE;
        } else {
            return;
        }
    }

    // 3. Obsługa upłynięcia czasu na potwierdzenie wiadomości
    if (buffer.available() && stopwatch.isTimeout()) {
        suspend();
        return;
    }

    // 4. Pomijamy obsługę pustych lub niekompletnych ramek
    if (frame.sensorCount == 0 || !frameComplete)
        return;

    // 5. Obsługa transmisji ramki
    switch (state) {
        case IDLE:

        case PREAMBLE:
            write(BB_START_BYTE);
            write(random());

            stopwatch.reset(BB_ECHO_TIMEOUT);
            state = FRAME_METADATA;
            break;
        case FRAME_METADATA:
            crc8.reset();
            write(address);
            write(seq++);
            write(frame.sensorCount);
            write(crc8.getCrc());

            stopwatch.reset(BB_ECHO_TIMEOUT);
            state = SENSOR_METADATA;
            break;
        case SENSOR_METADATA:
            crc8.reset();
            write(frame.sensors[currentSensor].address);
            write(frame.sensors[currentSensor].registerCount);
            write(crc8.getCrc());

            stopwatch.reset(BB_ECHO_TIMEOUT);
            state = REGISTER;
            break;
        case REGISTER:
            crc8.reset();
            for (size_t i = 0; i < frame.sensors[currentSensor].registerCount; i++) {
                write(frame.sensors[currentSensor].registers[i].address);
                write(frame.sensors[currentSensor].registers[i].data);
            }
            write(crc8.getCrc());
            currentSensor++;

            stopwatch.reset(BB_ECHO_TIMEOUT);
            state = currentSensor >= frame.sensorCount ? TERMINATOR : SENSOR_METADATA;
            break;
        case TERMINATOR:
            write(BB_STOP_BYTE);

            state = IDLE;
            break;
        default:
            return;
    }
}


void BroadcastSender::write(uint8_t data) {
    stream.write(data);
    buffer.write(data);
    crc8.update(data);
}

void BroadcastSender::suspend() {
    stopwatch.reset(random(BB_DELAY_MIN, BB_DELAY_MAX));
    buffer.clear();
    state = SUSPENDED;
}