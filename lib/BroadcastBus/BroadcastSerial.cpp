#include "BroadcastSerial.h"
#include "wiring_private.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler() {
    Serial3.IrqHandler();
}

BroadcastSerial::BroadcastSerial(Uart& stream, int serialIdx)
    : stream(stream), serialIdx(serialIdx) {
}

uint8_t BroadcastSerial::receiveCmd() {
    if (rxBuffer.cmd == 0) {
        if (available()) {
#ifdef BB_DEBUG
            Serial.print("[S");
            Serial.print(serialIdx);
            Serial.print("]");
            Serial.print("[BB] Available CMD buffer ");
            Serial.println(available());
#endif
            int cmd = read();
#ifdef BB_DEBUG
            Serial.print("[S");
            Serial.print(serialIdx);
            Serial.print("]");
            Serial.print("[BB] Reading CMD = 0x");
            Serial.println(cmd, HEX);
#endif
            cmd ^= BB_MASK_START;
            if (cmd & 0xF0) {
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] CMD incorrect = 0x");
                Serial.println(cmd, HEX);
#endif
            } else {
                rxBuffer.cmd = cmd;
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] CMD correct = 0x");
                Serial.println(cmd, HEX);
#endif
            }
        }
    } else {
        if (rxStopwatch.isTimeout()) {
            reset();
#ifdef BB_DEBUG
            Serial.print("[S");
            Serial.print(serialIdx);
            Serial.print("]");
            Serial.println("[BB] Timeout");
#endif
        }
    }
    return rxBuffer.cmd;
}

bool BroadcastSerial::receiveData(size_t len) {
    while (available()) {
#ifdef BB_DEBUG
        Serial.print("[S");
        Serial.print(serialIdx);
        Serial.print("]");
        Serial.print("[BB] Available buffer ");
        Serial.println(available());
#endif
        if (rxBuffer.length < len) {
            int data = readData();
            rxBuffer.data[rxBuffer.length++] = data;
#ifdef BB_DEBUG
            Serial.print("[S");
            Serial.print(serialIdx);
            Serial.print("]");
            Serial.print("[BB] Read data = ");
            Serial.println(data);
#endif
        } else if (!rxBuffer.hasCrc) {
            rxBuffer.crc = read();
            rxBuffer.hasCrc = true;
            if (rxBuffer.crc != rxCrc.getCrc()) {
                reset();
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] CRC is incorrect | crc=");
                Serial.println(rxBuffer.crc);
#endif
                return false;
            } else {
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] CRC is correct | crc=");
                Serial.println(rxBuffer.crc);
#endif
            }
        } else {
            uint8_t cmd = read() ^ BB_MASK_STOP;
            if (rxBuffer.cmd == cmd) {
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] STOP byte correct | cmd =");
                Serial.println(cmd);
#endif
                return true;
            } else {
#ifdef BB_DEBUG
                Serial.print("[S");
                Serial.print(serialIdx);
                Serial.print("]");
                Serial.print("[BB] STOP byte incorrect | cmd =");
                Serial.println(cmd);
#endif
                reset();
            }
        }
    }

#ifdef BB_DEBUG
    if (rxBuffer.cmd != 0) {
        Serial.print("[S");
        Serial.print(serialIdx);
        Serial.print("]");
        Serial.print("[BB] Reading frame | cmd=");
        Serial.print(rxBuffer.cmd);
        Serial.print(", len=");
        Serial.print(rxBuffer.length);
        Serial.print(", crc=");
        Serial.println(rxBuffer.crc);
    }
#endif
    return false;
}

void BroadcastSerial::reset() {
    rxBuffer.cmd = 0;
    rxCrc.reset();
    memset(&rxBuffer, 0, sizeof(bb_buffer));
}

void BroadcastSerial::begin(unsigned long baudRate) {
    stream.begin(baudRate);
    while (!stream) {}
}

int BroadcastSerial::available() {
    return stream.available();
}

uint8_t BroadcastSerial::read() {
    if (available())
        rxStopwatch.reset();

    return stream.read();
}

uint8_t BroadcastSerial::readData() {
    int data = read();
    rxCrc.update(data);
    return data;
}

bool BroadcastSerial::readChecksum() {
    int data = read();
    return data == rxCrc.getCrc();
}

void BroadcastSerial::resetRxChecksum() {
    rxCrc.reset();
}

void BroadcastSerial::write(uint8_t data) {
    stream.write(data);
}

void BroadcastSerial::flush() {
//    stream.flush();
}
