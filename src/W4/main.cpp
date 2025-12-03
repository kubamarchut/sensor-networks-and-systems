#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>
#include <Stopwatch.h>
#include <Wire.h>

#define NODE_ID 0x04
#define MAX_SENSORS 16

BroadcastBus bus = BroadcastBus();

bb_sensor_frame frames[MAX_SENSORS];
size_t frames_length = 0;
size_t frames_ptr = 0;

Stopwatch finish_stopwatch = Stopwatch(1000);
bool uart1_finish = false, uart2_finish = false;

uint8_t seq = 0;

bool acqRequested = false;

void onI2CReceive(int len) {
    if (len < 1) return;

    uint8_t cmd = Wire.read();
    
    if (cmd == 'A') {
        seq = Wire.read();
        acqRequested = true;

        frames_length = 0;
        frames_ptr = 0;
        bus.sendRequest(seq);
        finish_stopwatch.reset();
    }
}

void onI2CRequest() {
    if (frames_ptr < frames_length) {
        Wire.write((uint8_t*) (&frames[frames_ptr++]), sizeof(bb_sensor_frame));
    } else {
        if ((uart1_finish && uart2_finish) || finish_stopwatch.isTimeout()) {
            Wire.write(1);
        } else {
            Wire.write(0);
        }
    }
}

void bb_frame_finish() {
    Serial.println("Finished frame");
    bb_print_frame(frames[frames_length]);
    frames_length = (frames_length + 1) % BB_MAX_SENSORS;
}

void setup() {
    Wire.begin(NODE_ID);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.begin(115200);
    bus.begin();
    while (!Serial);

    Serial.println("Setup W4");
}

void loop() {
    switch (bus.bSerial1.receiveCmd()) {
        case BB_MASK_SEN:
            if (bus.bSerial1.receiveData(sizeof(bb_sensor_frame))) {
                memcpy(&frames[frames_length], bus.bSerial1.rxBuffer.data, sizeof(bb_sensor_frame));
                bb_frame_finish();
                bus.bSerial1.reset();
            }
            break;
        case BB_MASK_FIN:
            if (bus.bSerial1.receiveData(2)) {
//                uint8_t finishSeq = bus.bSerial1.rxBuffer.data[0];
//                uint8_t sensorCount = bus.bSerial1.rxBuffer.data[1];
                uart1_finish = true;
            }
            break;
    }
    switch (bus.bSerial2.receiveCmd()) {
        case BB_MASK_SEN:
            if (bus.bSerial2.receiveData(sizeof(bb_sensor_frame))) {
                memcpy(&frames[frames_length], bus.bSerial2.rxBuffer.data, sizeof(bb_sensor_frame));
                bb_frame_finish();
                bus.bSerial2.reset();
            }
            break;
        case BB_MASK_FIN:
            if (bus.bSerial2.receiveData(2)) {
//                uint8_t finishSeq = bus.bSerial2.rxBuffer.data[0];
//                uint8_t sensorCount = bus.bSerial2.rxBuffer.data[1];
                uart2_finish = true;
            }
            break;
    }

    delay(1);
}