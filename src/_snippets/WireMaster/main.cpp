#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDR 0x15

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Wire Master - pomyślna inicjalizacja");
}

void loop() {
    Serial.print("Wysyłanie zapytania o dane...");

    uint32_t startTime = millis();
    uint8_t bytesReceived = Wire.requestFrom(SLAVE_ADDR, 1);
    uint32_t duration = millis() - startTime;

    Serial.print("\tWysłano (czas=");
    Serial.print(duration);
    Serial.print(" ms, bytes=");
    Serial.print(bytesReceived);
    Serial.println(")");

    if (Wire.available()) {
        uint8_t data = Wire.read();
        Serial.print("Odebrano dane: 0x");
        Serial.println(data, HEX);
    } else {
        Serial.println("Nie odebrano danych");
    }

    delay(5000);
}