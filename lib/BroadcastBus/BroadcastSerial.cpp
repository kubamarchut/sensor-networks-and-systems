#include "BroadcastSerial.h"
#include "wiring_private.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void Serial3_init() {
    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
}

void SERCOM3_Handler() {
    Serial3.IrqHandler();
}
