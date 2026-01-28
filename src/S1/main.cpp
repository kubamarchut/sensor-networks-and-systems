#include <Arduino.h>
#include <WirelessCommunication.h>
#include <Indicator.h>
#include <TCS3200.h>

#ifndef NODE_ADDR
#define NODE_ADDR 0x07
#endif

//#define FAKE_SENSOR
#define S0_PIN 4
#define S1_PIN 3
#define S2_PIN 2
#define S3_PIN 1
#define OUT_PIN 5

//#define LORA_DEBUG

#define POWER_PIN 12

#ifndef FAKE_SENSOR
TCS3200 tcs3200(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);
#endif

Indicator indicator(9, 10, 11);

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

#ifndef FAKE_SENSOR
void performMeasurement(RGB &color) {
  RGBColor rgb = tcs3200.read_rgb_color();
  color.r = (uint8_t) constrain(rgb.red,   0, 255);
  color.g = (uint8_t) constrain(rgb.green, 0, 255);
  color.b = (uint8_t) constrain(rgb.blue,  0, 255);
}
#endif

void readData() {
  #ifdef FAKE_SENSOR
  updateRGBWithDelta(currentColor, 5);
  #endif
  #ifndef FAKE_SENSOR
  digitalWrite(POWER_PIN, HIGH);
  delay(100);
  performMeasurement(currentColor);
  digitalWrite(POWER_PIN, LOW);
  #endif
}

void setup() {
  randomSeed(analogRead(A0));
  indicator.begin();
  indicator.setColor(Indicator::RED);
  delay(1000);
  indicator.setColor(Indicator::GREEN);
  delay(1000);
  indicator.setColor(Indicator::BLUE);
  delay(1000);
  indicator.setColor(Indicator::OFF);
  
  Serial.begin(115200);
  #ifdef LORA_DEBUG
    while (!Serial){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  #endif

  #ifndef FAKE_SENSOR
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  tcs3200.begin();
  tcs3200.frequency_scaling(TCS3200_OFREQ_2P);

  // delay(3000);
  // Serial.println("Calibrating white...");
  
  // uint32_t r = tcs3200.read_red();
  // uint32_t g = tcs3200.read_green();
  // uint32_t b = tcs3200.read_blue();

  // Serial.print("R: "); Serial.print(r);
  // Serial.print("  G: "); Serial.print(g);
  // Serial.print("  B: "); Serial.println(b);
  
  // Serial.println("White calibration done");
  
  // delay(3000);
  // Serial.println("Calibrating black...");

  // r = tcs3200.read_red();
  // g = tcs3200.read_green();
  // b = tcs3200.read_blue();
 
  // Serial.print("R: "); Serial.print(r);
  // Serial.print("  G: "); Serial.print(g);
  // Serial.print("  B: "); Serial.println(b);

  tcs3200.calibrate_light(488, 771, 673);
  tcs3200.calibrate_dark(3190, 5601, 5004);
  tcs3200.calibrate();

  digitalWrite(POWER_PIN, LOW);
  #endif

  Serial.print("S");
  Serial.print(1);
  Serial.println(" uruchomiony");

  if (!radio.begin(NODE_ADDR, ROLE_SLAVE, &indicator)) {
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

      Serial.print("pkt.to: ");
      Serial.println(pkt.to);

      readData();
      WirelessPacket dataResponse;
      memset(&dataResponse, 0, sizeof(WirelessPacket));
      dataResponse.type = PKT_RES;
      memset(dataResponse.payload, 0, 8);
      dataResponse.length = 3;
      dataResponse.payload[0] = 0x01;
      dataResponse.payload[1] = currentColor.r;
      dataResponse.payload[2] = 0x02;
      dataResponse.payload[3] = currentColor.g;
      dataResponse.payload[4] = 0x03;
      dataResponse.payload[5] = currentColor.b;
      memcpy(dataResponse.initialTrace, pkt.trace, MAX_NODES);
      memset(dataResponse.trace, 0, MAX_NODES);
      dataResponse.trace[0] = NODE_ADDR;
      dataResponse.to = 0x01;
      dataResponse.seq = pkt.seq;
      radio.send(dataResponse);
  }
}