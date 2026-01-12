#include <Arduino.h>
#include "Crc8.h"
#include "morslib.h"
#include "WirelessCommunication.h"

WirelessCommunication radio;

void TC5_Handler() {
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  radio.onSlotStartISR();
}

void guardISR() {
    radio.onGuardEndISR();
}

void setupTimers(Tc* tc, uint32_t ms) {
  // 1. Enable clock for TC3 (APBC Mask)
  PM->APBCMASK.reg |= PM_APBCMASK_TC5;

  // 2. Configure GCLK0 (48MHz) for TC3
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | 
                                  GCLK_CLKCTRL_GEN_GCLK0 | 
                                  GCLK_CLKCTRL_ID_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  // 3. Configure TC3 (16-bit, Prescaler 1024, Match Freq Mode)
  tc->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV1024 |
                           TC_CTRLA_PRESCSYNC_PRESC;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY);

  // 4. Calculate Compare Value
  // CPU: 48MHz, Prescaler: 1024 -> 46875 Hz tick
  // Target: ms
  uint32_t ccValue = (46875 * ms) / 1000;
  
  tc->COUNT16.CC[0].reg = (uint16_t)ccValue;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY);

  // 5. Enable Interrupt
  tc->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_EnableIRQ(TC5_IRQn);

  // 6. Enable TC3
  tc->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY);
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
  Serial.print(NODE_ADDR - 1);
  Serial.println(" uruchomiony");
  
  if (!radio.begin(NODE_ADDR, ROLE_MASTER)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

  Serial.println("Inicjalizacja radio udana");
  
  setupTimers(TC5, 1000*1000);

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
