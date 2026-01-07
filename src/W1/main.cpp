#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"

WirelessCommunication radio;

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

#define DEST_ADDR 0x02            // Roboczo do usunięcia przez MJ

unsigned long lastPing = 0;

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
  Serial.print(NODE_ADDR);
  Serial.println(" uruchomiony");
  
  if (!radio.begin()) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
  
  mymors.queue('s');
}

void loop() {
  mymors.handle();
  
  if (millis() - lastPing > 30000) {
    lastPing = millis();
    Serial.println("[>] Sending ping");
    radio.sendPing(NODE_ADDR, DEST_ADDR);
  }

  // ---- RECEIVE ----
  WirelessPacket rx;
  if (radio.receive(rx)) {
    if (rx.to == NODE_ADDR) {
      Serial.print("[<] Packet from W");
      Serial.println(rx.from);

      if (rx.type == 1) {
        Serial.print("\tPING received from ");
        Serial.println(rx.from);
        Serial.println("[>] Sending pong");
        radio.sendPong(NODE_ADDR, rx.from);
      }
      else if (rx.type == 2) {
        Serial.print("\tPONG received from ");
        Serial.println(rx.from);
      }
    }
    else {
      Serial.println("[<] Received msg not addressed for this node");
    }
  }

  //delay(10);
}
