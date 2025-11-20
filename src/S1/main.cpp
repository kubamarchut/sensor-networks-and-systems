#include <Arduino.h>
#include <Wire.h>

// adres I2C czujnika koloru
#define SENSOR_ADDR 0x10

// rejestry dla składowych RGB
#define REG_CNT 3
#define REG_R 0x01
#define REG_G 0x02
#define REG_B 0x03

// do symulacji - co ile ms zmiana
#define COLOR_UPDATE_FREQ 1000

// timeout obsluga protokolu I2C
#define COMM_STATE_TIMEOUT 500

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
  Wire.begin(SENSOR_ADDR);
  Wire.onRequest(onI2CRequest);
  Serial.begin(9600);
  while (!Serial);
  Serial.println("S1 (czujnik koloru) uruchomiony");
}

void loop() {
  if (millis() - lastColorChangeTime >= COLOR_UPDATE_FREQ){
    redVal   = random(0, 255);
    greenVal = random(0, 255);
    blueVal  = random(0, 255);

    lastColorChangeTime = millis();
  }

  if (communicationState == 1){
    if (millis() - lastCommunicationStateChange >= COMM_STATE_TIMEOUT){
      communicationState = 0;
    }
  }
}
