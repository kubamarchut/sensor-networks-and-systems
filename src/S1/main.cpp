#include <Arduino.h>
#include <WirelessCommunication.h>

#ifndef NODE_ADDR
#define NODE_ADDR 0x07
#endif

WirelessCommunication radio;

void setup() {
  Serial.begin(115200);
  #ifdef LORA_DEBUG:
    while (!Serial){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  #endif

  Serial.print("S");
  Serial.print(1);
  Serial.println(" uruchomiony");
}

void loop() {

}