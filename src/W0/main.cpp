
#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

WirelessCommunication radio;

static uint32_t lastSync = 0;

morslib mymors(LED_BUILTIN, 200);

void setup() {
  mymors.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);   
  while(!Serial) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
  }
  Serial.print("W");
  Serial.print(NODE_ADDR - 1);
  Serial.println(" uruchomiony");
  
  if (!radio.begin(NODE_ADDR, ROLE_MASTER, 500)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

  Serial.println("Inicjalizacja radio udana");

  mymors.queue('s');
}

void loop() {
  mymors.handle();
  radio.poll();
}
