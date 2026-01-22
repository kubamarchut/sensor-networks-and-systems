#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"

WirelessCommunication radio;

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

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
  
  if (!radio.begin(NODE_ADDR, ROLE_SLAVE)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
  
  mymors.queue('s');
}

void loop() {
  mymors.handle();

  radio.poll();
  WirelessPacket pkt;
  if (radio.receive(pkt)) {
      Serial.print("Received packet ");
      Serial.println(pkt.type);

      WirelessPacket dataResponse;
      dataResponse.type = PKT_RES;
      dataResponse.length = 0;
      radio.send(dataResponse);
  }
}
