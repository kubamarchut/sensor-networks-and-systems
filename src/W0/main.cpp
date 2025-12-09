#include <Arduino.h>
#include <Wire.h>
#include <BroadcastBus.h>
#include <Stopwatch.h>
#include "morslib.h"
#include "wiring_private.h"

#define MAX_ZONE_2_NODES 10   // Maks. liczba węzłów I2C
#define MAX_REGS_PER_SENSOR 8 // Maks. liczba rejestrów na sensor
#define DATA_PULL_FREQ 10000  // Częst. odpytywania o dane
#define MAX_REGS 16*8        // Maks. liczba czujników w sieci

#define I2C_STATUS_EMPTY    0x51
#define I2C_STATUS_DATA     0x62
#define I2C_STATUS_FINISHED 0x73
#define I2C_DEBUG 1

#define WIRE1_SDA 0
#define WIRE1_SCL 1
#define WIRES_LEN 2

TwoWire Wire1(&sercom3, WIRE1_SDA, WIRE1_SCL);
TwoWire *Wires[2] = {&Wire, &Wire1};

morslib mymors(LED_BUILTIN, 200);

typedef struct __attribute__((packed)) {
    uint8_t node_count;
    uint8_t node_addr[MAX_ZONE_2_NODES];
    bb_sensor_frame sensor;
} bb_node_frame;

bb_sensor_frame frames[MAX_REGS];
size_t frames_length;
bool printed_frames = false;
Stopwatch request_stopwatch = Stopwatch(DATA_PULL_FREQ);
Stopwatch data_stopwatch = Stopwatch(500);

struct nodeInfo {
    size_t wireIdx;
    uint8_t address;
    bool finished;
};
nodeInfo nodes[MAX_ZONE_2_NODES];
uint8_t nodeCount = 0;
uint8_t seq = 0;

void print_frames() {
    if (printed_frames)
        return;

    for (size_t i = 0; i < frames_length; i++) {
        bb_print_frame_compact(frames[i]);
    }
    printed_frames = true;
    frames_length = 0;
}

void addNode(size_t wireIdx, uint8_t addr) {
    if (nodeCount < MAX_ZONE_2_NODES) {
        Serial.print("Znaleziono węzeł (seq=");
        Serial.print(seq);
        Serial.print(") dla adresu ");
        Serial.print("0x");
        Serial.println(addr, HEX);
        nodes[nodeCount].wireIdx = wireIdx;
        nodes[nodeCount].address = addr;
        nodes[nodeCount].finished = false;
        nodeCount++;
    } else {
        Serial.println("Brak miejsca na kolejne czujniki!");
    }
}

// Faza wykrywania urządzeń I2C
void discoverI2CDevices() {
    Serial.println("Skanowanie magistrali I2C...");
    nodeCount = 0;
    seq++;

    for (size_t wireIdx = 0; wireIdx < WIRES_LEN; wireIdx++) {
        TwoWire *wire = Wires[wireIdx];
        for (uint8_t addr = 1; addr < 127; addr++) {
            if (addr == 0x60 || addr == 0x6B)
                continue;

            wire->beginTransmission(addr);
            wire->write('A');
            wire->write(seq);
            uint8_t error = wire->endTransmission();

            if (error == 0) {
                addNode(wireIdx, addr);
            }
        }
    }

    Serial.print("Znaleziono ");
    Serial.print(nodeCount);
    Serial.println(" węzłów sąsiednich");
}

void requestData(uint8_t i) {
    Crc8 crc;
    uint8_t addr = nodes[i].address;
    TwoWire *wire = Wires[nodes[i].wireIdx];
    wire->requestFrom(addr, sizeof(bb_sensor_frame) + 2);
    bb_sensor_frame frame_buffer;

    uint8_t first_byte = wire->read();

    if (first_byte == I2C_STATUS_EMPTY) {
#ifdef I2C_DEBUG
        Serial.print("[");
        Serial.print(i);
        Serial.print("]");
        Serial.println("[I2C] No data available");
#endif
        return;
    } else if (first_byte == I2C_STATUS_FINISHED) {
        nodes[i].finished = 1;
#ifdef I2C_DEBUG
        Serial.print("[");
        Serial.print(i);
        Serial.print("]");
        Serial.println("[I2C] Data finished");
#endif
        bool allFinished = true;
        for (size_t n = 0; n < nodeCount; n++) {
            if (!nodes[n].finished) {
                allFinished = false;
                break;
            }
        }

        if (allFinished)
            print_frames();

        return;
    } else if (first_byte == I2C_STATUS_DATA) {
#ifdef I2C_DEBUG
        Serial.print("[");
        Serial.print(i);
        Serial.print("]");
        Serial.println("[I2C] Found frame");
#endif
        wire->readBytes((uint8_t *) &frame_buffer, sizeof(bb_sensor_frame));
        if (seq != frame_buffer.seq) {
#ifdef I2C_DEBUG
            Serial.print("[");
            Serial.print(i);
            Serial.print("]");
            Serial.println("[I2C] Incorrect seq");
#endif
            return;
        }

        crc.calculate((uint8_t *) &frame_buffer, sizeof(bb_sensor_frame));
        int readCrc = wire->read();
        if (crc.getCrc() != readCrc) {
#ifdef I2C_DEBUG
            Serial.print("[");
            Serial.print(i);
            Serial.print("]");
            Serial.println("[I2C] Incorrect CRC");
#endif
            return;
        }

        int frame_idx = frames_length;
        for (size_t index = 0; index < frames_length; index++) {
            if (frames[index].sensor_addr == frame_buffer.sensor_addr) {
                frame_idx = index;
                break;
            }
        }

        memcpy(&frames[frame_idx], &frame_buffer, sizeof(bb_sensor_frame));
        frames_length = max(min(frame_idx + 1, MAX_REGS), frames_length);
#ifdef I2C_DEBUG
        Serial.print("[");
        Serial.print(i);
        Serial.print("]");
        Serial.println("[I2C] Saved frame");
#endif
    } else {
#ifdef I2C_DEBUG
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.println("[I2C] Received unknown frame");
#endif
    }

    // Czyszczenie nadmiarowych bajtów
    while (wire->available())
        wire->read();
}

void setup() {
    mymors.begin();
    Serial.begin(9600);
    while (!Serial) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Wire.begin();
    Wire1.begin();
    Serial.println("W0 uruchomiony...");
    mymors.queue('s');
    pinPeripheral(WIRE1_SDA, PIO_SERCOM);
    pinPeripheral(WIRE1_SCL, PIO_SERCOM);
}

void loop() {
    mymors.handle();

    if (data_stopwatch.isTimeout()) {
        for (int i = 0; i < nodeCount; i++) {
            if (!nodes[i].finished) {
                requestData(i);
            }
        }
        data_stopwatch.reset();
    }

    if (request_stopwatch.isTimeout()) {
        print_frames();
        discoverI2CDevices();
        printed_frames = false;
        mymors.queue('n', 1);
        request_stopwatch.reset();
    }
}
