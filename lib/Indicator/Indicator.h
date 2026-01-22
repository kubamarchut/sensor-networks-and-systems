#ifndef INDICATOR_H
#define INDICATOR_H

#include <Arduino.h>

class Indicator {
  public:
    enum Color {
      OFF,
      RED,
      GREEN,
      YELLOW
    };

    Indicator(uint8_t pinR, uint8_t pinG);
    void begin();
    void setColor(Color color);

  private:
    uint8_t _pinR;
    uint8_t _pinG;

    void off();
    void red();
    void green();
    void yellow();
};

#endif
