#include <Arduino.h>
#include <Wire.h>
#include "morslib.h"

#define NODE_ID             0xFF   // ID tego węzła
#define MAX_ZONE_2_NODES    10     // Maks. liczba węzłów I2C
#define MAX_DATA_SIZE       32     // Maks. rozmiar danych z jednego urządzenia
#define MAX_REGS_PER_SENSOR 8      // Maks. liczba rejestrów na sensor
#define DATA_PULL_FREQ      10000  // Częst. odpytywania o dane

morslib mymors(LED_BUILTIN, 200);

struct SensorInfo {
  uint8_t address;
};

SensorInfo sensors[MAX_ZONE_2_NODES];
uint8_t sensorCount = 0;

unsigned long lastRequestTime = 0;

// Faza wykrywania urządzeń I2C
void discoverI2CDevices() {
  Serial.println("Skanowanie magistrali I2C...");
  sensorCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    //mymors.handle();
    if (addr == 0x60 || addr == 0x6B)
      continue;
    
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      if (sensorCount < MAX_ZONE_2_NODES) {
        sensors[sensorCount].address = addr;
        Serial.println(addr, HEX);
        sensorCount++;
      }
    }
    //delay(10);
  }

  Serial.print("Znaleziono ");
  Serial.print(sensorCount);
  Serial.println(" węzłów sąsiednich");  
}

void setup() {
  mymors.begin();
  Serial.begin(9600);    
  Wire.begin();
  while(!Serial);

  Serial.println("W0 uruchomiony");
  mymors.queue('s');
  mymors.queue('t');
  mymors.queue('e');
  delay(1000);
  lastRequestTime = millis();
}

void loop() {
  mymors.handle();
  
  if (millis() - lastRequestTime >= DATA_PULL_FREQ) {
    discoverI2CDevices();
    lastRequestTime = millis();
    mymors.queue('?');
    mymors.queue('n');

  }

  //delay(10);
}
