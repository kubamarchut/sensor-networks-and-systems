#include <Arduino.h>
#include <Wire.h>

#define NODE_ID 0x03            // ID tego węzła
#define MAX_SENSORS 10          // Maks. liczba urządzeń I2C
#define MAX_DATA_SIZE 32        // Maks. rozmiar danych z jednego urządzenia
#define MAX_REGS_PER_SENSOR 8   // Maks. liczba rejestrów na sensor

#define START_BYTE  0xAA        // UART znak start
#define STOP_BYTE   0x55        // UART znak stop

struct RegisterPair {
  uint8_t regAddr;
  uint8_t regValue;
};

// Struktura bufora dla danych z czujnikow
struct SensorFrame {
  uint8_t nodeAddr;
  uint8_t seqNum;
  uint8_t sensorAddr;
  uint8_t regCount;
  RegisterPair regs[MAX_REGS_PER_SENSOR];
};

SensorFrame buffer[MAX_SENSORS];
uint8_t bufferCount = 0;

// Dane dla zadania I2C
uint8_t requestedSensor = 0;
bool hasRequest = false;

//void onI2CReceive(int numBytes);
//void onI2CRequest();

void storeSensorFrame(SensorFrame &sf) {
  for (uint8_t i = 0; i < bufferCount; i++) {
    if (buffer[i].sensorAddr == sf.sensorAddr) {
      buffer[i] = sf;
      return;
    }
  }
  if (bufferCount < MAX_SENSORS) {
    buffer[bufferCount++] = sf;
  }
}

void processFrame(uint8_t *frame, uint8_t length) {
  if (length < 3) return; // za krótka ramka

  uint8_t senderAddr = frame[0];
  uint8_t seqNum = frame[1];
  uint8_t sensorCount = frame[2];

  uint8_t pos = 3;
  
  if (sensorCount == 0 || sensorCount > MAX_SENSORS) return;

  // oblicz CRC  
  uint8_t recvCRC = frame[length - 1];
  uint8_t calcCRC = 0;
  for (uint8_t i = 0; i < length - 1; i++) {
    calcCRC += frame[i];
  }
  if (calcCRC != recvCRC) {
    Serial.println("CRC error");
    return;
  }

  // przetworzenie bloku dla sensora
  for (uint8_t i = 0; i < sensorCount; i++) {
    if (pos + 1 >= length - 1) break; // za mało bajtów

    SensorFrame sf;
    sf.nodeAddr = senderAddr;
    sf.seqNum = seqNum;
    sf.sensorAddr = frame[pos++];
    sf.regCount = frame[pos++];

    if (sf.regCount > MAX_REGS_PER_SENSOR) sf.regCount = MAX_REGS_PER_SENSOR;

    for (uint8_t r = 0; r < sf.regCount; r++) {
      if (pos + 1 >= length - 1) break;

      sf.regs[r].regAddr  = frame[pos++];
      sf.regs[r].regValue = frame[pos++];
    }
    
    Serial.print("Otrzymano ramke od ");
    Serial.print(senderAddr, HEX);
    Serial.print(" (seq ");
    Serial.print(seqNum);
    Serial.print(") dane z ");
    Serial.print(sensorCount);
    Serial.println(" sensorow");
  }
  
}

void parseUARTFrames() {
  static bool receiving = false;
  static uint8_t frame[128];
  static uint8_t idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    if (!receiving) {
      if (b == START_BYTE) { // poczatek ramki
        receiving = true;
        idx = 0;
      }
      continue;
    }

    if (receiving) {
      if (b == STOP_BYTE) { // koniec ramki
        processFrame(frame, idx);
        receiving = false;
        idx = 0;
      }
      else {
        if (idx < sizeof(frame)) {
          frame[idx++] = b;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(9600);    
  while(!Serial);
  //Wire.begin(NODE_ID);
  //Wire.onReceive(onI2CReceive);
  //Wire.onRequest(onI2CRequest);

  Serial.println("W1 uruchomiony");
}

void loop() {
  parseUARTFrames();
}
