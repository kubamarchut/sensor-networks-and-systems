#include <Arduino.h>
#include <Wire.h>
#include "morslib.h"

morslib mymors(LED_BUILTIN, 200);

// adres I2C czujnika koloru
//define NODE_ADDR 0x20
#define DEBUG 1

// rejestry dla składowych RGB
#define REG_CNT 3
#define REG_R (NODE_ADDR + 0x01)
#define REG_G (NODE_ADDR + 0x02)
#define REG_B (NODE_ADDR + 0x03)

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

void performMeasurments()
{
    redVal   = random(0, 255);
    greenVal = random(0, 255);
    blueVal  = random(0, 255);
    if(DEBUG)
    {
      Serial.print("DEBUG | R: "); Serial.print(redVal);
      Serial.print("  G: "); Serial.print(greenVal);
      Serial.print("  B: "); Serial.println(blueVal);
    }

    lastColorChangeTime = millis();
    mymors.queue('p');
}

// wysłanie danych po I2C – format: [liczba rejestrów] lub n * ([klucz][wartość])
void onI2CRequest() {
  uint8_t registers = REG_CNT;
  performMeasurments();

  if (communicationState == 0){
    Wire.write(registers);

    communicationState = 1;
    lastCommunicationStateChange = millis();
  }
  else if (communicationState == 1){
    Wire.write(REG_R); Wire.write(redVal);
    Wire.write(REG_G); Wire.write(greenVal);
    Wire.write(REG_B); Wire.write(blueVal);
    mymors.queue('r');

    communicationState = 0;
  }
}

void setup() {
  Wire.begin(NODE_ADDR);
  Wire.onRequest(onI2CRequest);
  Serial.begin(9600);
  //while (!Serial);
  Serial.print("S");
  Serial.print(NODE_ADDR);
  randomSeed(analogRead(A0));
  mymors.begin();
  mymors.queue('s');
  Serial.println(" (czujnik) uruchomiony");
}

void loop() {
  mymors.handle();

  if (communicationState == 1){
    if (millis() - lastCommunicationStateChange >= COMM_STATE_TIMEOUT){
      communicationState = 0;
    }
  }
}
