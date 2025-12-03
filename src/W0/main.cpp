#include <Arduino.h>
#include <Wire.h>
#include <BroadcastConsts.h>
#include <Stopwatch.h>
#include "morslib.h"

#define MAX_ZONE_2_NODES 10   // Maks. liczba węzłów I2C
#define MAX_REGS_PER_SENSOR 8 // Maks. liczba rejestrów na sensor
#define DATA_PULL_FREQ 10000  // Częst. odpytywania o dane
#define MAX_SENSORS 16        // Maks. liczba czujników w sieci

morslib mymors(LED_BUILTIN, 200);

bb_sensor_frame frames[MAX_SENSORS];
size_t frames_length;
bool printed_frames = false;
Stopwatch request_stopwatch = Stopwatch(DATA_PULL_FREQ);
Stopwatch data_stopwatch = Stopwatch(100);

struct nodeInfo
{
    uint8_t address;
    bool finished;
};
nodeInfo nodes[MAX_ZONE_2_NODES];
uint8_t nodeCount = 0;
uint8_t seq = 0;

void print_frames() {
    if (printed_frames)
        return;

    for (size_t i = 0; i < frames_length; i++) {
        bb_print_frame(frames[i]);
    }
    printed_frames = true;
}

void addNode(uint8_t addr)
{
  if (nodeCount < MAX_ZONE_2_NODES)
  {
    nodes[nodeCount].address = addr;
    nodes[nodeCount].finished = false;
    nodeCount++;
  } else {
    Serial.println("Brak miejsca na kolejne czujniki!");
  }
}

// Faza wykrywania urządzeń I2C
void discoverI2CDevices()
{
  Serial.println("Skanowanie magistrali I2C...");
  nodeCount = 0;
    seq++;

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    // mymors.handle();
    if (addr == 0x60 || addr == 0x6B)
      continue;

    Wire.beginTransmission(addr);
    Wire.write('A');
    Wire.write(seq);
    uint8_t error = Wire.endTransmission();

    if (error == 0)
    {
      addNode(addr);
      Serial.print("0x");
      Serial.println(addr, HEX);
    }
  }

  Serial.print("Znaleziono ");
  Serial.print(nodeCount);
  Serial.println(" węzłów sąsiednich");
}

void requestData(uint8_t i)
{
    uint8_t addr = nodes[i].address;
  size_t bytes = Wire.requestFrom(addr, sizeof(bb_sensor_frame));
  if (bytes < sizeof(bb_sensor_frame)) {
      Wire.readBytes((uint8_t*) (&frames[frames_length++]), sizeof(bb_sensor_frame));
  } else {
      nodes[i].finished = Wire.read();
      if (nodes[i].finished) {
          print_frames();
      }
  }

  // Czyszczenie nadmiarowych bajtów
  while (Wire.available()) Wire.read();
}

void setup()
{
  mymors.begin();
  Serial.begin(9600);
  Wire.begin();
  while (!Serial);

  Serial.println("W0 uruchomiony");
  mymors.queue('s');
}

void loop()
{
  mymors.handle();

  if (data_stopwatch.isTimeout()) {
      for (int i = 0; i < nodeCount; i++) {
          if (!nodes[i].finished)
              requestData(i);
      }
      data_stopwatch.reset();
  }

  if (request_stopwatch.isTimeout()) {
    print_frames();
    discoverI2CDevices();
    printed_frames = false;
    mymors.queue('n', 1);
    request_stopwatch.reset();
  }
}
