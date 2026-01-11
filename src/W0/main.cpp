#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"
#include "SAMDTimerInterrupt.h"

SAMDTimer ITimer(TIMER_TC3);
WirelessCommunication radio;

void slotISR() {
    radio.onSlotStartISR();
}

void guardISR() {
    radio.onGuardEndISR();
}

void setupTimers() {
    ITimer.attachInterruptInterval(1000 * 1000, slotISR);
}

static uint32_t lastSync = 0;


#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

morslib mymors(LED_BUILTIN, 200);

void setup() {
  mymors.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);   
  while(!Serial) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
  }
  Serial.print("W");
  Serial.print(NODE_ADDR);
  Serial.println(" uruchomiony");
  
  if (!radio.begin(NODE_ADDR, ROLE_MASTER)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
  
  mymors.queue('s');
}

void loop() {
  mymors.handle();
  

  if (millis() - lastSync > 5000) {
    WirelessPacket pkt{};
    pkt.to = 0xFF;
    pkt.type = PKT_TIME_SYNC;
    pkt.seq++;
    pkt.length = 4;
    lastSync = millis();
    memcpy(pkt.payload, &lastSync, 4);
    pkt.trace[0] = NODE_ADDR;
    radio.send(pkt);
    lastSync = millis();
  }

  radio.poll();
}
