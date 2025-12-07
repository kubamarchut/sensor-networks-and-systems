#include <Arduino.h>
#include <Wire.h>
#include <TCS3200.h>

// adres I2C czujnika koloru
// #define NODE_ADDR 0x10

// rejestry dla składowych RGB
#define REG_CNT 3
#define REG_R (NODE_ADDR + 0x01)
#define REG_G (NODE_ADDR + 0x02)
#define REG_B (NODE_ADDR + 0x03)

// do symulacji - co ile ms zmiana
#define COLOR_UPDATE_FREQ 5000

// timeout obsluga protokolu I2C
#define COMM_STATE_TIMEOUT 500

#define DEBUG 1
#define S0_PIN 6
#define S1_PIN 7
#define S2_PIN 8
#define S3_PIN 9
#define OUT_PIN 5
#define PIN_POWER 10

TCS3200 tcs3200(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);

// czas ostatniej zmiany
unsigned long lastColorChangeTime = 0;

// wartości pomiarow (symulacja lub odczyt z TCS3200)
uint8_t redVal = 0;
uint8_t greenVal = 0;
uint8_t blueVal = 0;

// kontroler stanu do komunikacji I2C
volatile uint8_t communicationState = 0;
unsigned long lastCommunicationStateChange = 0;

// wysłanie danych po I2C – format: [liczba rejestrów] lub n * ([klucz][wartość])
void onI2CRequest() {
  uint8_t registers = REG_CNT;

  if (communicationState == 0){
    Wire.write(registers);

    communicationState = 1;
    lastCommunicationStateChange = millis();
  }
  else if (communicationState == 1){
    Wire.write(REG_R); Wire.write(redVal);
    Wire.write(REG_G); Wire.write(greenVal);
    Wire.write(REG_B); Wire.write(blueVal);

    communicationState = 0;
  }
}

void setup() {
  Wire.begin(NODE_ADDR);
  Wire.onRequest(onI2CRequest);
  Serial.begin(9600);
  while (!Serial);
  Serial.print("S");
  Serial.print(NODE_ADDR);
  Serial.println(" (czujnik) uruchomiony");
  //Serial.println("kalibracja");

  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);
  
  // Init TCS3200
  tcs3200.begin();
  tcs3200.frequency_scaling(TCS3200_OFREQ_2P);
  
  // delay(3000);
  // Serial.println("Calibrating white...");
  
  // uint32_t r = tcs3200.read_red();
  // uint32_t g = tcs3200.read_green();
  // uint32_t b = tcs3200.read_blue();

  tcs3200.calibrate_light(1367, 1993, 1723);
  // Serial.print("R: "); Serial.print(r);
  // Serial.print("  G: "); Serial.print(g);
  // Serial.print("  B: "); Serial.println(b);
  
  // Serial.println("White calibration done");
  
  // delay(3000);
  // Serial.println("Calibrating black...");

  // r = tcs3200.read_red();
  // g = tcs3200.read_green();
  // b = tcs3200.read_blue();

  tcs3200.calibrate_dark(10269, 19202, 17411);  
  // Serial.print("R: "); Serial.print(r);
  // Serial.print("  G: "); Serial.print(g);
  // Serial.print("  B: "); Serial.println(b);
  
  
  tcs3200.calibrate();
  Serial.println("Calibration complete");
  digitalWrite(PIN_POWER, LOW);
}

void loop() {
  tcs3200.loop();   // required for library operation
  if (millis() - lastColorChangeTime >= COLOR_UPDATE_FREQ){
    digitalWrite(PIN_POWER, HIGH);
    delay(100);

    // Read sensor
    RGBColor rgb = tcs3200.read_rgb_color();
    redVal   = (uint8_t) constrain(rgb.red,   0, 255);
    greenVal = (uint8_t) constrain(rgb.green, 0, 255);
    blueVal  = (uint8_t) constrain(rgb.blue,  0, 255);

    // Debug to Serial
    if (DEBUG){
      Serial.print("DEBUG | R: "); Serial.print(redVal);
      Serial.print("  G: "); Serial.print(greenVal);
      Serial.print("  B: "); Serial.println(blueVal);
    }

    digitalWrite(PIN_POWER, LOW);
    lastColorChangeTime = millis();
  }

  if (communicationState == 1){
    if (millis() - lastCommunicationStateChange >= COMM_STATE_TIMEOUT){
      communicationState = 0;
    }
  }
}
