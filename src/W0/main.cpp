#include <Arduino.h>
#include <Wire.h>
#include "morslib.h"

#define NODE_ID 0xFF          // ID tego węzła
#define MAX_ZONE_2_NODES 10   // Maks. liczba węzłów I2C
#define MAX_DATA_SIZE 32      // Maks. rozmiar danych z jednego urządzenia
#define MAX_REGS_PER_SENSOR 8 // Maks. liczba rejestrów na sensor
#define DATA_PULL_FREQ 10000  // Częst. odpytywania o dane
#define MAX_SENSORS 16        // Maks. liczba czujników w sieci

#define liczbaBajtowDoPobrania = MAX_SENSORS * MAX_REGS_PER_SENSOR * 2;

morslib mymors(LED_BUILTIN, 200);

struct SensorInfo
{
  //uint8_t address;
  uint8_t registers[MAX_REGS_PER_SENSOR];
  uint8_t data[MAX_REGS_PER_SENSOR];
};

struct nodeInfo
{
  uint8_t address;
};

nodeInfo nodes[MAX_ZONE_2_NODES];
uint8_t nodeCount = 0;

SensorInfo sensors[MAX_SENSORS];

unsigned long lastRequestTime = 0;

void addNode(uint8_t addr)
{
  if (nodeCount < MAX_ZONE_2_NODES)
  {
    nodes[nodeCount].address = addr;
    nodeCount++;
  }
  else
  {
    Serial.println("Brak miejsca na kolejne czujniki!");
  }
}

// Faza wykrywania urządzeń I2C
void discoverI2CDevices()
{
  Serial.println("Skanowanie magistrali I2C...");
  nodeCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    // mymors.handle();
    if (addr == 0x60 || addr == 0x6B)
      continue;

    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {

      addNode(addr);
      Serial.print("0x");
      Serial.println(addr, HEX);
      // if (sensorCount < MAX_ZONE_2_NODES) {
      //   sensors[sensorCount].address = addr;
      //   sensorCount++;
      // }
    }
  }

  Serial.print("Znaleziono ");
  Serial.print(nodeCount);
  Serial.println(" węzłów sąsiednich");
}

void zapytajODane(uint8_t addr)
{
  uint8_t odp = 0;
  Wire.requestFrom(addr, 1, false);
  while (Wire.available())
    odp = Wire.read();
  if (odp != 0 && odp != 255)
  {
    for (int j = 0; j < MAX_SENSORS;j++)
      for (int i = 0; i < MAX_REGS_PER_SENSOR; i++)
      {
        Wire.requestFrom(addr, 2, false);
        sensors[j].registers[i]=Wire.read();
        sensors[j].data[i]=Wire.read();
    }
  }
}

void printDane(){
  Serial.print("reg");
      Serial.print("\t");
      Serial.println("data");
  for (int i = 0; i < MAX_SENSORS;i++){
    Serial.println("--------------");
    Serial.print("sensor ");
    Serial.println(i);
    for (int j = 0; j < MAX_REGS_PER_SENSOR;j++){
      Serial.print(sensors[i].registers[j]);
      Serial.print("\t");
      Serial.println(sensors[i].registers[j]);
    }
  }
}

void setup()
{
  lastRequestTime = millis();
  mymors.begin();
  Serial.begin(9600);
  Wire.begin();
  while (!Serial)
    ;

  Serial.println("W0 uruchomiony");
  mymors.queue('s');
}

void loop()
{
  mymors.handle();

  if (millis() - lastRequestTime >= DATA_PULL_FREQ)
  {
    discoverI2CDevices();
    lastRequestTime = millis();
    mymors.queue('n', 1);
    // for (int i = 0; i < nodeCount; i++)
    // {
    //   Serial.println(nodes[i].address, HEX);
    // }

    zapytajODane(nodes[1].address);
    printDane();
  }
}
