#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>
#include <Stopwatch.h>
#include <Wire.h>

#define NODE_ID 0x04

bb_sensor_frame frames[BB_MAX_SENSORS];
size_t current_frame_idx = 0;
uint8_t seq = 0;
BroadcastBus bus = BroadcastBus();
Stopwatch seqStopwatch = Stopwatch(2000);

int receivedSeq = -1;
bool acqRequested = false;
uint8_t dataLen = 0;

void onI2CReceive(int len) {
    if (len < 2) return;

    uint8_t cmd = Wire.read();
    
    if (cmd == 'A') {
        uint8_t seq = Wire.read();
        receivedSeq = seq;
        acqRequested = true;

        Serial.print("[I2C] ACQ request seq=");
        Serial.println(receivedSeq);
    }
}

void onI2CRequest() {
    if (dataLen > 0) {
        Wire.write(dataBuffer, dataLen);
    } else {
        Wire.write((uint8_t)0x00);
    }
}

void bb_frame_finish() {
    Serial.println("Finished frame");
    Serial.print("\tNode address = 0x");
    Serial.println(frames[current_frame_idx].node_addr, HEX);
    Serial.print("\tSensor address = 0x");
    Serial.println(frames[current_frame_idx].sensor_addr, HEX);
    Serial.print("\tSeq = ");
    Serial.println(frames[current_frame_idx].seq);
    Serial.print("\tFlags = 0b");
    Serial.println(frames[current_frame_idx].flags, BIN);
    Serial.print("\tRegisters length =");
    Serial.println(frames[current_frame_idx].regs_len, DEC);
    for (size_t i = 0; i < BB_MAX_REGISTERS; i++) {
        Serial.print("\tRegister[");
        Serial.print(i, DEC);
        Serial.print("] = 0x");
        Serial.print(frames[current_frame_idx].regs[i].addr, HEX);
        Serial.print(" - ");
        Serial.println(frames[current_frame_idx].regs[i].data);
    }

    current_frame_idx = (current_frame_idx + 1) % BB_MAX_SENSORS;
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
    if (seqStopwatch.isTimeout()) {
        bus.sendRequest(seq);
        Serial.print("Next seq = ");
        Serial.println(seq);
        seqStopwatch.reset();
        seq++;
    }

    switch (bus.bSerial1.receiveCmd()) {
        case BB_MASK_SEN:
            if (bus.bSerial1.receiveData(sizeof(bb_sensor_frame))) {
                memcpy(&frames[current_frame_idx], bus.bSerial1.rxBuffer.data, sizeof(bb_sensor_frame));
                bb_frame_finish();
                bus.bSerial1.reset();
            }
            break;
    }
    delay(1);
}