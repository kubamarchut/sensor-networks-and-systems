#include <Arduino.h>
#include <Wire.h>

#define NODE_ID 0x01            // ID tego węzła master
#define DATA_PULL_FREQ 2000     // Częstotliwość odczytu (ms)
#define MAX_SENSORS 10          // Maks. liczba urządzeń I2C
#define MAX_DATA_SIZE 32        // Maks. rozmiar danych z jednego urządzenia

struct SensorInfo {
  uint8_t address;
};

SensorInfo sensors[MAX_SENSORS];
uint8_t sensorCount = 0;

unsigned long lastRequestTime = 0;

// Faza wykrywania urządzeń I2C
void discoverI2CDevices() {
  Serial.println("Skanowanie magistrali I2C...");
  sensorCount = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      if (sensorCount < MAX_SENSORS) {
        sensors[sensorCount].address = addr;
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
uint8_t readSensor(uint8_t addr, uint8_t *buffer) {
  uint8_t N = 0;

  // Zapytanie o rozmiarze 1 bajt
  Wire.requestFrom(addr, (uint8_t)1, (uint8_t)false);
  if (Wire.available()) {
    N = Wire.read();
  }
  else {
    return 0;
  }

  // Sprawdzenie poprawności rozmiaru
  if (N == 0 || N > MAX_DATA_SIZE) return 0;

  // Odczyt właściwych danych
  Wire.requestFrom(addr, N, (uint8_t)true);

  uint8_t bytesRead = 0;
  while (Wire.available() && bytesRead < N) {
    buffer[bytesRead++] = Wire.read();
  }
  
  return bytesRead;
}

// Wysłanie ramki UART (ramka binarna)
void sendUARTFrame(uint8_t sensorAddr, uint8_t *data, uint8_t dataLen) {
  uint8_t startByte = 0xAA;
  uint8_t stopByte  = 0x55;

  // Prosty CRC: suma modulo 256
  uint8_t crc = NODE_ID + sensorAddr;
  for (uint8_t i = 0; i < dataLen; i++) crc += data[i];

  Serial.write(startByte);
  Serial.write(NODE_ID);
  Serial.write(sensorCount);
  Serial.write(sensorAddr | 0x01);
  Serial.write(dataLen);

  // Dane surowe
  for (uint8_t i = 0; i < dataLen; i++) Serial.write(data[i]);

  Serial.write(crc);
  Serial.write(stopByte);
}

void setup() {
  Wire.begin();          
  Serial.begin(9600);    
  while(!Serial);
  Serial.println("W1 uruchomiony");
  discoverI2CDevices();
}

void loop() {
  if (millis() - lastRequestTime >= DATA_PULL_FREQ) {
    for (uint8_t i = 0; i < sensorCount; i++) {
      uint8_t addr = sensors[i].address;
      uint8_t buffer[MAX_DATA_SIZE];
      
      uint8_t bytesRead = readSensor(addr, buffer);
      if (bytesRead > 0) {
        sendUARTFrame(addr, buffer, bytesRead);
      }
    }
    lastRequestTime = millis();
  }
}
