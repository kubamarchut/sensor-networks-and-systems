#include <Arduino.h>
#include <Wire.h>
#include <BroadcastBus.h>
#include <Stopwatch.h>
#include "morslib.h"
#include "wiring_private.h"

struct PingPacket {
  uint8_t from;
  uint8_t to;
  uint8_t type; // 1 = PING, 2 = PONG
};

#define DEST_ID 2

morslib mymors(LED_BUILTIN, 200);

void setup() {
    mymors.begin();
    Serial.begin(9600);
    while (!Serial) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Serial.println("W0 uruchomiony...");
    mymors.queue('s');
}

void loop() {
    mymors.handle();    
}
