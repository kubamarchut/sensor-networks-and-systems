#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>

BroadcastBus bus = BroadcastBus();
Stopwatch seqStopwatch = Stopwatch();

bb_sensor_frame frame = {
    .node_addr = 0xD9,
    .sensor_addr = 0,
    .seq = 1,
    .flags = 0b11010010,
    .regs_len = 2,
    .regs = {
    {.addr = 0x11, .data = 41},
    {.addr = 0x12, .data = 223},
    {.addr = 0x1A, .data = 125},
    {.addr = 0x1B, .data = 96},
    {.addr = 0x1C, .data = 84},
    {.addr = 0x2A, .data = 72},
    {.addr = 0x2B, .data = 193},
    {.addr = 0x2C, .data = 147},
    },
};

void setup() {
    Serial.begin(115200);
    bus.begin();
    while (!Serial);

    Serial.println("Setup W2");
}

void loop() {
    switch (bus.bSerial1.receiveCmd()) {
        case BB_MASK_REQ:
            if (bus.bSerial1.receiveData(1)) {
                frame.seq = bus.bSerial1.rxBuffer.data[0];
                Serial.print("New seq = ");
                Serial.print(frame.seq);
                Serial.print(", time = ");
                Serial.print(seqStopwatch.getElapsed());
                Serial.println();
                seqStopwatch.reset();
                bus.bSerial1.reset();
            }
            break;
    }

    bus.sendSensor(frame);

    Serial.println("Sent frame");
    delay(500);
}

