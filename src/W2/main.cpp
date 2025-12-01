#include <Arduino.h>
#include <BroadcastBus.h>
#include <Crc8.h>
#include "wiring_private.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

bb_sensor_frame frame = {
    .node_addr = 0xD9,
    .sensor_addr = 0,
    .seq = 1,
    .flags = 0b11010010,
    .regs_len = 2,
    .regs = {
    {.addr = 0x11, .data = 41},
    {.addr = 0x12, .data = 223},
    {.addr = 0x1A, .data = 0},
    {.addr = 0x1B, .data = 0},
    {.addr = 0x1C, .data = 0},
    {.addr = 0x2A, .data = 0},
    {.addr = 0x2B, .data = 0},
    {.addr = 0x2C, .data = 0},
    },
};
Crc8 crc8;

void bb_write(uint8_t data) {
    Serial.println(data);
    Serial3.write(data);
    crc8.update(data);
}

void SERCOM3_Handler() {
    Serial3.IrqHandler();
}

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);
    while (!Serial);
    while (!Serial3);

    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
}

void loop() {
    bb_write(BB_MASK_START | BB_MASK_RES);
    crc8.reset();
    bb_write(frame.node_addr);
    bb_write(frame.sensor_addr);
    bb_write(frame.seq);
    Serial.println("Sent incomplete frame");
    delay(100);

    bb_write(BB_MASK_START | BB_MASK_RES);
    crc8.reset();
    bb_write(frame.node_addr);
    bb_write(frame.sensor_addr);
    bb_write(frame.seq);
    bb_write(frame.flags);
    bb_write(frame.regs_len);
    for (size_t i = 0; i < BB_MAX_REGISTERS; i++) {
        frame.regs[i].data = random(0, 255);
        bb_write(frame.regs[i].addr);
        bb_write(frame.regs[i].data);
    }
    bb_write(crc8.getCrc());
    bb_write(BB_MASK_STOP | BB_MASK_RES);
    Serial3.flush();
    Serial.println("Sent frame");
    delay(500);
}

