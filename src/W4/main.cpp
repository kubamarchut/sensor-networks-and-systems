#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>
#include <Stopwatch.h>
#include <Wire.h>

#define NODE_ID 0x04
#define MAX_SENSORS 16

#define I2C_STATUS_EMPTY    0x51
#define I2C_STATUS_DATA     0x62
#define I2C_STATUS_FINISHED 0x73

BroadcastBus bus = BroadcastBus();

bb_sensor_frame frames[MAX_SENSORS];
size_t frames_length = 0;
size_t frames_ptr = 0;

Stopwatch finish_stopwatch = Stopwatch(10000);
bool uart1_finish = false, uart2_finish = false;

uint8_t seq = 0;

void onI2CReceive(int len) {
    if (len < 1) return;

    uint8_t cmd = Wire.read();

    Serial.print("REQUEST cmd=");
    Serial.print(cmd);

    if (cmd == 'A') {
        Serial.print(",");
        seq = Wire.read();
        Serial.print(" seq=");
        Serial.print(seq);

        frames_length = 0;
        frames_ptr = 0;
        bus.sendRequest(seq);
        finish_stopwatch.reset();
        uart1_finish = false;
        uart2_finish = false;
    }
}

void onI2CRequest() {
    if (frames_ptr < frames_length) {
        Serial.print("Wysyłanie ramki frames_ptr=");
        Serial.println(frames_ptr);

        Wire.write(I2C_STATUS_DATA);

        Crc8 crc;
        crc.calculate((uint8_t*) (&frames[frames_ptr]), sizeof(bb_sensor_frame));
        Wire.write((uint8_t*) (&frames[frames_ptr]), sizeof(bb_sensor_frame));
        Wire.write(crc.getCrc());
        frames_ptr += 1;
    } else {
        Serial.print("Brak nowej ramki finished=");
        if ((uart1_finish && uart2_finish) || finish_stopwatch.isTimeout()) {
            Serial.println(1);
            Wire.write(I2C_STATUS_FINISHED);
        } else {
            Serial.println(0);
            Wire.write(I2C_STATUS_EMPTY);
        }
    }
}

void bb_frame_finish() {
    Serial.println("Finished frame");
    bb_print_frame(frames[frames_length]);
    frames_length = (frames_length + 1) % BB_MAX_SENSORS;
}

void setup() {
    Serial.begin(115200);
    bus.begin();
    while(!Serial) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Wire.begin(NODE_ID);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    Serial.println("Setup W4");
}

void loop() {
    switch (bus.bSerial1.receiveCmd()) {
        case BB_MASK_SEN:
            if (bus.bSerial1.receiveData(sizeof(bb_sensor_frame))) {
                Serial.println("Serial1 - sensor data");
                memcpy(&frames[frames_length], bus.bSerial1.rxBuffer.data, sizeof(bb_sensor_frame));
                bb_frame_finish();
                bus.bSerial1.reset();
            }
            break;
        case BB_MASK_FIN:
            if (bus.bSerial1.receiveData(2)) {
                Serial.println("Serial1 - finish");
//                uint8_t finishSeq = bus.bSerial1.rxBuffer.data[0];
//                uint8_t sensorCount = bus.bSerial1.rxBuffer.data[1];
                uart1_finish = true;
                bus.bSerial1.reset();
            }
            break;
    }
    switch (bus.bSerial2.receiveCmd()) {
        case BB_MASK_SEN:
            if (bus.bSerial2.receiveData(sizeof(bb_sensor_frame))) {
                Serial.println("Serial2 - sensor data");
                memcpy(&frames[frames_length], bus.bSerial2.rxBuffer.data, sizeof(bb_sensor_frame));
                bb_frame_finish();
                bus.bSerial2.reset();
            }
            break;
        case BB_MASK_FIN:
            if (bus.bSerial2.receiveData(2)) {
                Serial.println("Serial2 - finish");
//                uint8_t finishSeq = bus.bSerial2.rxBuffer.data[0];
//                uint8_t sensorCount = bus.bSerial2.rxBuffer.data[1];
                uart2_finish = true;
                bus.bSerial2.reset();
            }
            break;
    }

    delay(1);
}