#include <Arduino.h>
#include <Wire.h>
#include "morslib.h"

#define NODE_ID 0x03            // ID tego węzła
#define MAX_SENSORS 16          // Maks. liczba urządzeń I2C
#define MAX_PAIR_SIZE 2         // 1 byte + 1 byte 

struct RegDataPair {
    uint8_t reg;  
    uint8_t value;
};

void onI2CRequest() {
  
}


void setup() {
  Wire.begin(NODE_ID);
  Wire.onRequest(onI2CRequest);
  Serial.begin(9600);
  while(!Serial);

  Serial.println("W1 uruchomiony");
}

void loop() {
  
}
