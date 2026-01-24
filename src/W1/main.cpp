#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"

WirelessCommunication radio;

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

Indicator indicator(9, 10, 11);
morslib mymors(LED_BUILTIN, 200);

void setup() {
  mymors.begin();

  randomSeed(analogRead(A0));
  indicator.begin();
  indicator.setColor(Indicator::RED);
  delay(1000);
  indicator.setColor(Indicator::GREEN);
  delay(1000);
  indicator.setColor(Indicator::BLUE);
  delay(1000);
  indicator.setColor(Indicator::OFF);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Serial.print("W");
  Serial.print(NODE_ADDR-1);
  Serial.println(" uruchomiony");
  
  if (!radio.begin(NODE_ADDR, ROLE_SLAVE, &indicator)) {
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
      memset(&dataResponse, 0, sizeof(WirelessPacket));
      dataResponse.type = PKT_RES;
      memset(dataResponse.payload, 0, 8);
      dataResponse.length = 0;
      memcpy(dataResponse.initialTrace, pkt.trace, MAX_NODES);
      memset(dataResponse.trace, 0, MAX_NODES);
      dataResponse.trace[0] = NODE_ADDR;
      dataResponse.to = 0x01;
      dataResponse.seq = pkt.seq;
      radio.send(dataResponse);
  }
}
