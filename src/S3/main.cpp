#include <Arduino.h>
#include <Wire.h>

#define DEBUG 1
// adres I2C czujnika koloru
// #define SENSOR_ADDR 0x30

// rejestry dla składowych RGB
#define REG_CNT 2
#define REG_R (NODE_ADDR + 0x01)
#define REG_G (NODE_ADDR + 0x02)

// do symulacji - co ile ms zmiana
#define DATA_UPDATE_FREQ 2000

// timeout obsluga protokolu I2C
#define COMM_STATE_TIMEOUT 500

// czas ostatniej zmiany
unsigned long lastDataChangeTime = 0;

// wartości pomiarow (symulacja lub odczyt z TCS3200)
uint8_t tempVal = 0;
uint8_t humVal = 0;

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
    Wire.write(REG_R); Wire.write(tempVal);
    Wire.write(REG_G); Wire.write(humVal);

    communicationState = 0;
  }
}

void setup() {
  Wire.begin(NODE_ADDR);
  Wire.onRequest(onI2CRequest);
  Serial.begin(9600);
  while (!Serial);
  Serial.print("S");
  Serial.print(NODE_ADDR);
  Serial.println(" (czujnik) uruchomiony");
}

void loop() {
  if (millis() - lastDataChangeTime >= DATA_UPDATE_FREQ){
    tempVal = random(15, 25);
    humVal  = random(40, 50);
    
    if (DEBUG){
      Serial.print("DEBUG | TEMP: "); Serial.print(tempVal);
      Serial.print(" HUM: "); Serial.println(humVal);
    }

    lastDataChangeTime = millis();
  }

  if (communicationState == 1){
    if (millis() - lastCommunicationStateChange >= COMM_STATE_TIMEOUT){
      communicationState = 0;
    }
  }
}
