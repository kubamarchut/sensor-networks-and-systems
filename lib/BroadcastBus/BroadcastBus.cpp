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
    write(BB_MASK_START | cmd);
    txCrc.reset();
}

void BroadcastBus::endTransmission() {
    if (txCmd == 0)
        return;

    write(txCrc.getCrc());
    write(BB_MASK_STOP | txCmd);
    flush();
    txCmd = 0;
}

void BroadcastBus::sendRequest(uint8_t seq) {
    startTransmission(BB_MASK_REQ);
    writeData(seq);
    endTransmission();
}

void BroadcastBus::sendSensor(bb_sensor_frame& data) {
    startTransmission(BB_MASK_SEN);
    writeData((uint8_t*)&data, sizeof(data));
    endTransmission();
}

void BroadcastBus::sendFinish(uint8_t seq, uint8_t len) {
    startTransmission(BB_MASK_FIN);
    writeData(seq);
    writeData(len);
    endTransmission();
}

void BroadcastBus::write(uint8_t data) {
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