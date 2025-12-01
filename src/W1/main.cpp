#include <Arduino.h>
#include <Wire.h>
#include "Crc8.h"

#define NODE_ID 0x01            // ID tego węzła master
#define DATA_PULL_FREQ 2000     // Częstotliwość odczytu (ms)
#define MAX_SENSORS   8         // Maks. liczba urządzeń I2C
#define MAX_DATA_SIZE 8         // Maks. liczba rejestów z jednego urządzenia


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
  if (Wire.available()) {
    N = Wire.read();
  }
  else {
    return 0;
  }

  N *= 2;
  Serial.print("N=");
  Serial.println(N);
  // Sprawdzenie poprawności rozmiaru
  if (N == 0 || N > MAX_DATA_SIZE) return 0;

  // Odczyt właściwych danych
  Wire.requestFrom(addr, N);

  uint8_t bytesRead = 0;
  Serial.print("|");
  while (Wire.available() && bytesRead < N) {
    dataBuffer[i][bytesRead] = Wire.read();

    Serial.print(dataBuffer[i][bytesRead], HEX);
    Serial.print("|");
    bytesRead++;
  }
  Serial.println();

  //sendUARTFrame();

  return bytesRead;
}

// Wysłanie ramki UART (ramka binarna)
void sendUARTFrame(uint8_t sensorAddr, uint8_t *data, uint8_t dataLen) {
  uint8_t frame[256];
  uint8_t pos = 0;
  
  frame[pos++] = 0;
  frame[pos++] = NODE_ID;
  frame[pos++] = seqNum++;
  frame[pos++] = sensorCount;

  // Prosty CRC: suma modulo 256
  uint8_t crc = 0;
  for (uint8_t i = 1; i < pos; i++) crc += data[i];
  frame[pos++] = crc;

  frame[pos++] = 0;

  Serial.write(frame, pos);
}

void acquireData(){
  discoverI2CDevices();
  for (uint8_t i = 0; i < sensorCount; i++) {
    uint8_t addr = sensors[i].address;
    
    uint8_t bytesRead = readSensor(addr, i);
    Serial.print("Odczytano wiad o rozmiarze: ");
    Serial.println(bytesRead);
  }
}

void setup() {
  Serial.begin(9600);    
  Serial1.begin(9600);    
  while(!Serial and !Serial1);
  Serial.println("W1 uruchomiony");
  Wire.begin();
  delay(1000);
}

void loop() {
  if (millis() - lastRequestTime >= DATA_PULL_FREQ) {
    acquireData();
    lastRequestTime = millis();
  }

  delay(10);
}
