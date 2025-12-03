#include "BroadcastBus.h"
#include <wiring_private.h>

void BroadcastBus::begin() {
    bSerial1.begin(BB_BAUD);
    bSerial2.begin(BB_BAUD);

    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
}

void BroadcastBus::startTransmission(uint8_t cmd) {
    txCmd = cmd;
#ifdef BB_DEBUG
    Serial.print("[BB] Start transmission cmd=0x");
    Serial.println(cmd, HEX);
#endif
    write(BB_MASK_START | cmd);
    txCrc.reset();
}

void BroadcastBus::endTransmission() {
#ifdef BB_DEBUG
    Serial.print("[BB] End transmission cmd=0x");
    Serial.println(txCmd, HEX);
#endif
    if (txCmd == 0)
        return;

    write(txCrc.getCrc());
    write(BB_MASK_STOP | txCmd);
    flush();
    txCmd = 0;
}

void BroadcastBus::sendRequest(uint8_t seq) {
#ifdef BB_DEBUG
    Serial.print("[BB] Request seq=0x");
    Serial.println(seq);
#endif
    startTransmission(BB_MASK_REQ);
    writeData(seq);
    endTransmission();
}

void BroadcastBus::sendSensor(bb_sensor_frame& data) {
#ifdef BB_DEBUG
    Serial.print("[BB] Send sensor addr=0x");
    Serial.println(data.sensor_addr, HEX);
#endif
    startTransmission(BB_MASK_SEN);
    writeData((uint8_t*)&data, sizeof(data));
    endTransmission();
}

void BroadcastBus::sendFinish(uint8_t seq, uint8_t len) {
#ifdef BB_DEBUG
    Serial.print("[BB] Finish seq=");
    Serial.println(seq);
#endif
    startTransmission(BB_MASK_FIN);
    writeData(seq);
    writeData(len);
    endTransmission();
}

void BroadcastBus::write(uint8_t data) {
#ifdef BB_DEBUG
    Serial.print("[BB] Write=");
    Serial.print(data);
    Serial.print(" | 0b");
    Serial.print(data, BIN);
    Serial.print(" | 0x");
    Serial.println(data, HEX);
#endif
    bSerial1.write(data);
    bSerial2.write(data);
}

void BroadcastBus::writeData(uint8_t data) {
    write(data);
    txCrc.update(data);
}

void BroadcastBus::writeData(uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        writeData(data[i]);
    }
}

void BroadcastBus::flush() {
    bSerial1.flush();
    bSerial2.flush();
}