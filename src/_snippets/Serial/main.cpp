/**
* SCHEMAT POŁĄCZEŃ
*
* +--------------------------+                +--------------------------+
* | MKR WAN 1310 (DEVICE A)  |                | MKR WAN 1310 (DEVICE B)  |
* +--------------------------+                +--------------------------+
* |                          |                |                          |
* | --- INTERFEJS 1 (Serial1) ------------------------------------------ |
* | Pin 13 (RX)              <--------------  Pin 14 (TX)                |
* | Pin 14 (TX)              -------------->  Pin 13 (RX)                |
* |                          |                |                          |
* | --- INTERFEJS 2 (Serial3) ------------------------------------------ |
* | Pin 1 (RX / PA23)        <--------------  Pin 0 (TX / PA22)          |
* | Pin 0 (TX / PA22)        -------------->  Pin 1 (RX / PA23)          |
* |                          |                |                          |
* | --- MASA ----------------------------------------------------------- |
* | GND                      <------------->  GND                        |
* +--------------------------+                +--------------------------+
*/

#include <Arduino.h>
#include "wiring_private.h"

Uart Serial3(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM3_Handler() {
    Serial3.IrqHandler();
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial3.begin(9600);

    pinPeripheral(1, PIO_SERCOM);
    pinPeripheral(0, PIO_SERCOM);
    Serial.println("Serial - pomyślna inicjalizacja");
}

void loop() {
    Serial1.print("Fx1tr");
    Serial3.print("BaXa9");

    delay(100);

    char buf[10];

    if (Serial1.available()) {
        int len = Serial1.readBytes(buf, 5);
        Serial.print("Serial1 RX: ");
        Serial.write(buf, len);
        Serial.println();
    }

    if (Serial3.available()) {
        int len = Serial3.readBytes(buf, 5);
        Serial.print("Serial3 RX: ");
        Serial.write(buf, len);
        Serial.println();
    }

    delay(5000);
}