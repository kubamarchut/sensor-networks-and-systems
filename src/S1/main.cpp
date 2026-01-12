#include <Arduino.h>

volatile bool timerFlag = false;

void () {
  // Acknowledge interrupt (Clear MC0 flag)
  TC3->COUNT16.INTFLAG.bit.MC0 = 1;
  timerFlag = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  setup_timer_tc3(1000); // 1000 ms
}

void loop() {
  if (timerFlag) {
    timerFlag = false;
    Serial.println("IRQ");
  }
}