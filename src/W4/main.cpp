#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>
#include <Stopwatch.h>

bb_sensor_frame frames[BB_MAX_SENSORS];
size_t current_frame_idx = 0;
size_t current_frame_ptr = 0;
bool current_frame_listening = false;
Crc8 crc8;
Stopwatch stopwatch(20);

void bb_frame_start() {
    Serial.println("Start frame");
    current_frame_ptr = 0;
    current_frame_listening = true;
    memset(&frames[current_frame_idx], 0, sizeof(bb_sensor_frame));
    crc8.reset();
}

void bb_frame_reset() {
    Serial.println("Reset frame");
    current_frame_ptr = 0;
    current_frame_listening = false;
    memset(&frames[current_frame_idx], 0, sizeof(bb_sensor_frame));
}

void bb_frame_finish() {
    Serial.println("Finished frame");
    Serial.print("\tNode address = 0x");
    Serial.println(frames[current_frame_idx].node_addr, HEX);
    Serial.print("\tSensor address = 0x");
    Serial.println(frames[current_frame_idx].sensor_addr, HEX);
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
    current_frame_ptr = 0;
    current_frame_listening = false;
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    while (!Serial);
    while (!Serial1);
    Serial.println("Setup");
}

void loop() {
    while (Serial1.available()) {
        uint8_t data = Serial1.read();
        uint8_t* frame_buffer = (uint8_t*) &frames[current_frame_idx];
        Serial.print("Reading data ");
        Serial.println(data);

        if (!current_frame_listening) {
            Serial.println("\tStarting frame");
            if (data == (BB_MASK_START | BB_MASK_SEN)) {
                Serial.println("\t\tSTART_RES");
                bb_frame_start();
                stopwatch.reset();
            }
        } else if (current_frame_ptr < sizeof(bb_sensor_frame)) {
            Serial.print("\t\tDATA ");
            Serial.println(current_frame_ptr);

            frame_buffer[current_frame_ptr++] = data;
            crc8.update(data);
            stopwatch.reset();
        } else if (current_frame_ptr == sizeof(bb_sensor_frame)) {
            Serial.println("\tCRC");
            if (crc8.getCrc() == data) {
                Serial.println("\t\tCRC correct");
                current_frame_ptr++;
                stopwatch.reset();
            } else {
                Serial.println("\t\tCRC incorrect");
                bb_frame_reset();
            }
        } else if (current_frame_ptr == sizeof(bb_sensor_frame)+1) {
            Serial.println("\tStopping frame");
            if (data == (BB_MASK_STOP | BB_MASK_SEN)) {
                Serial.println("\t\tSTOP_RES");
                bb_frame_finish();
            } else {
                bb_frame_reset();
            }
        }
    }
    if (current_frame_listening && stopwatch.isTimeout()) {
        bb_frame_reset();
    }
    delay(5);
}