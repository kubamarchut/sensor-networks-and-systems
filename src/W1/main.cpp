#include <Arduino.h>
#include <Wire.h>
#include "Crc8.h"
#include <BroadcastBus.h>
#include "morslib.h"

#define NODE_ADDR 0x02            // ID tego węzła master
#define DATA_PULL_FREQ 2000     // Częstotliwość odczytu (ms)
#define MAX_SENSORS   8         // Maks. liczba urządzeń I2C
#define MAX_DATA_SIZE 8         // Maks. liczba rejestów z jednego urządzenia

morslib mymors(LED_BUILTIN, 200);

int last_seq = -1;

BroadcastBus bus = BroadcastBus();
Stopwatch seqStopwatch = Stopwatch();

bb_sensor_frame frame = {
    .node_addr = NODE_ADDR,
    .sensor_addr = 0,
    .seq = 1,
    .flags = 0b11010010,
    .regs_len = 2,
    .regs = {
    {.addr = 0x11, .data = 41},
    {.addr = 0x12, .data = 223},
    {.addr = 0x1A, .data = 125},
    {.addr = 0x1B, .data = 96},
    {.addr = 0x1C, .data = 84},
    {.addr = 0x2A, .data = 72},
    {.addr = 0x2B, .data = 193},
    {.addr = 0x2C, .data = 147},
    },
};

// Struktury
struct SensorInfo {
  uint8_t address;
};

SensorInfo sensors[MAX_SENSORS];
uint8_t sensorCount = 0;

unsigned long lastRequestTime = 0;
uint8_t seqNum = 0;

uint8_t dataBuffer[MAX_SENSORS][MAX_DATA_SIZE*2];

void clearDataBuffer() {
    memset(dataBuffer, 0, sizeof(dataBuffer));
}

// Faza wykrywania urządzeń I2C
void discoverI2CDevices() {
  Serial.println("Skanowanie magistrali I2C...");
  sensorCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (addr == 0x60 || addr == 0x6B)
      continue;
    
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      if (sensorCount < MAX_SENSORS) {
        sensors[sensorCount].address = addr;
        Serial.println(addr, HEX);
        sensorCount++;
      }
    }
    delay(10);
  }

  Serial.print("Znaleziono ");
  Serial.print(sensorCount);
  Serial.println(" sensorów");  
}

// Odczyt danych z danego czujnika
uint8_t readSensor(uint8_t addr, uint8_t i) {
  clearDataBuffer();
  Serial.print("odczyt po adresie ");
  Serial.println(addr, HEX);
  uint8_t N = 0;

  // Zapytanie o rozmiarze 1 bajt
  Wire.requestFrom(addr, (uint8_t)1);
  frame.sensor_addr = addr;
  if (Wire.available()) {
    N = Wire.read();
  }
  else {
    return 0;
  }

  //N *= 2;
  Serial.print("N=");
  Serial.println(N);
  // Sprawdzenie poprawności rozmiaru
  if (N == 0 || N*2 > MAX_DATA_SIZE) return 0;

  frame.regs_len = N;

  // Odczyt właściwych danych
  int bytesRead = Wire.requestFrom(addr, N*2);
  for (int j = 0; j < N; j++)
  {
    if ((j+1)*2 > bytesRead)
      break;
    
    frame.regs[j].addr = Wire.read();
    frame.regs[j].data = Wire.read();
  }

  seqStopwatch.reset();

  bus.sendSensor(frame);

  return bytesRead;
}

void acquireData(){
  Serial.println("pobieranie danych");
  discoverI2CDevices();
  for (uint8_t i = 0; i < sensorCount; i++) {
    uint8_t addr = sensors[i].address;
    
    uint8_t bytesRead = readSensor(addr, i);
    Serial.print("Odczytano wiad o rozmiarze: ");
    Serial.println(bytesRead);
  }
  bus.sendFinish(frame.seq, sensorCount);
}

void setup() {
  mymors.begin();
    pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);   
  /*while(!Serial) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
  }*/
  Wire.begin();
  bus.begin();
  Serial.print("W");
  Serial.print(NODE_ADDR);
  Serial.println(" uruchomiony");
  mymors.queue('s');
}

void loop() {
  mymors.handle();
  switch (bus.bSerial1.receiveCmd()) {
    case BB_MASK_REQ:
        if (bus.bSerial1.receiveData(1)) {
          frame.seq = bus.bSerial1.rxBuffer.data[0];
          if (frame.seq != last_seq){
            acquireData();
            mymors.queue('a');
            mymors.queue('e');
            last_seq = frame.seq;
          }
          bus.bSerial1.reset();
        }
        break;
      }
      switch (bus.bSerial2.receiveCmd()) {
        case BB_MASK_REQ:
        if (bus.bSerial2.receiveData(1)) {
          frame.seq = bus.bSerial2.rxBuffer.data[0];
          if (frame.seq != last_seq){
            acquireData();
            mymors.queue('a');
            mymors.queue('i');
            last_seq = frame.seq;
          }
          bus.bSerial2.reset();
        }
        break;
    }

  //if (millis() - lastRequestTime >= DATA_PULL_FREQ) {
  //  
  //  lastRequestTime = millis();
  //}

  delay(10);
}
