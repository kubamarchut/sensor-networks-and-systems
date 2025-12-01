#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDR 0x15

void requestEvent();

void setup() {
    Serial.begin(115200);
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);

    Serial.println("Wire Slave - pomyślna inicjalizacja");
}

void loop() {
    delay(10);
}

void requestEvent() {
    Serial.println("[ISR] Odebrano zapytanie od mastera.");
    Serial.println("\tPrzytrzymanie magistrali...");

    // Trzymamy Mastera
    delay(3000);

    // Wysłanie odpowiedzi, którą powinien odebrać Master po X sekundach
    Wire.write(0xAA);
    Serial.println("\tOdpowiedź odesłana.");
}