#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include <SPI.h>
#include <LoRa.h>

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

#define DEST_ADDR 0x02            // Roboczo do usunięcia przez MJ

struct PingPacket {
  uint8_t from;
  uint8_t to;
  uint8_t type;
};

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
  
  if(!LoRa.begin(8681E5)) {
    Serial.println("LoRa init - niepowodzenie");
  }
  Serial.println("LoRa init - udane");
  
  mymors.queue('s');
}

void sendPing() {
  PingPacket pkt = { NODE_ADDR, DEST_ADDR, 1 };
  Serial.println("PING sending");
  
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();
  
  Serial.println("PING sent");
}

void sendPong(uint8_t dest) {
  PingPacket pkt = { NODE_ADDR, dest, 2 };
  Serial.println("PONG sending");

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();

  Serial.println("PONG sent");
}

void loop() {
  mymors.handle();
  
  if (millis() - lastPing > 2000) {
    lastPing = millis();
    sendPing();
  }

  // ---- RECEIVE ----
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(PingPacket)) {
    PingPacket pkt;
    LoRa.readBytes((uint8_t*)&pkt, sizeof(pkt));

    if (pkt.to != NODE_ADDR) return;

    if (pkt.type == 1) {
      Serial.print("PING received from ");
      Serial.println(pkt.from);
      sendPong(pkt.from);
    }
    else if (pkt.type == 2) {
      Serial.print("PONG received from ");
      Serial.println(pkt.from);
    }
  }

  //delay(10);
}
