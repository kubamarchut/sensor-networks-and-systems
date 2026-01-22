#include "Indicator.h"

Indicator::Indicator(uint8_t pinR, uint8_t pinG, uint8_t pinB) {
  _pinR = pinR;
  _pinG = pinG;
  _pinB = pinB;
}

void Indicator::begin() {
  pinMode(_pinR, OUTPUT);
  pinMode(_pinG, OUTPUT);
  pinMode(_pinB, OUTPUT);
  off();
}

void Indicator::setColor(Color color) {
  switch (color) {
    case OFF:    off();    break;
    case RED:    red(true);    break;
    case GREEN:  green(true);  break;
    case BLUE:   blue(true);   break;
    case YELLOW: red(true); green(true); break;
  }
}

void Indicator::off() {
  digitalWrite(_pinR, HIGH);
  digitalWrite(_pinG, HIGH);
  digitalWrite(_pinB, HIGH);
}

void Indicator::red(bool state) {
  digitalWrite(_pinR, state ? LOW : HIGH);
}

void Indicator::green(bool state) {
  digitalWrite(_pinG, state ? LOW : HIGH);
}

void Indicator::blue(bool state) {
  digitalWrite(_pinB, state ? LOW : HIGH);
}