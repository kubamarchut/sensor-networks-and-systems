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
  
  if (!radio.begin(NODE_ADDR, ROLE_RELAY)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
  
  mymors.queue('s');
}

void loop() {
  mymors.handle();

  radio.poll();
  WirelessPacket pkt;
  radio.receive(pkt);
}
