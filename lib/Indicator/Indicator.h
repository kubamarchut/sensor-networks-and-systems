#ifndef INDICATOR_H
#define INDICATOR_H

#include <Arduino.h>

class Indicator {
  public:
    enum Color {
      OFF,
      RED,
      GREEN,
      BLUE,
      YELLOW
    };

    Indicator(uint8_t pinR, uint8_t pinG, uint8_t pinB);
    void begin();
    void setColor(Color color);

    void off();
    void red(bool state);
    void green(bool state);
    void blue(bool state);

  private:
    uint8_t _pinR;
    uint8_t _pinG;
    uint8_t _pinB;
};

#endif
