#include <Arduino.h>
#include <WirelessCommunication.h>
#include <Indicator.h>

#ifndef NODE_ADDR
#define NODE_ADDR 0x07
#endif

Indicator indicator(9, 10);

WirelessCommunication radio;

struct RGB {
  int r;
  int g;
  int b;
};

RGB currentColor = {128, 128, 128};

void updateRGBWithDelta(RGB &color, int maxDelta) {
  color.r = constrain(color.r + random(-maxDelta, maxDelta + 1), 0, 255);
  color.g = constrain(color.g + random(-maxDelta, maxDelta + 1), 0, 255);
  color.b = constrain(color.b + random(-maxDelta, maxDelta + 1), 0, 255);
}

void readData() {
  delay(100);
  updateRGBWithDelta(currentColor, 5);
}

void setup() {
  randomSeed(analogRead(A0));
  indicator.begin();
  indicator.setColor(Indicator::RED);
  delay(1000);
  indicator.setColor(Indicator::GREEN);
  delay(1000);
  indicator.setColor(Indicator::YELLOW);
  delay(1000);
  indicator.setColor(Indicator::OFF);
  
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

  if (!radio.begin(NODE_ADDR, ROLE_SLAVE, 500)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

  Serial.println("Inicjalizacja radio udana");
}

void loop() {
  radio.poll();

  WirelessPacket pkt;
  if (radio.receive(pkt)) {
      Serial.print("Received packet ");
      Serial.println(pkt.type);

      readData();
      WirelessPacket dataResponse;
      dataResponse.type = PKT_RES;
      dataResponse.length = 3;
      dataResponse.payload[0] = 0x01;
      dataResponse.payload[1] = currentColor.r;
      dataResponse.payload[2] = 0x02;
      dataResponse.payload[3] = currentColor.g;
      dataResponse.payload[4] = 0x03;
      dataResponse.payload[5] = currentColor.b;
      radio.send(dataResponse);
  }
}