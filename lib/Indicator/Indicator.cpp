#include "Indicator.h"

Indicator::Indicator(uint8_t pinR, uint8_t pinG) {
  _pinR = pinR;
  _pinG = pinG;
}

void Indicator::begin() {
  pinMode(_pinR, OUTPUT);
  pinMode(_pinG, OUTPUT);
  off();
}

void Indicator::setColor(Color color) {
  switch (color) {
    case OFF:    off();    break;
    case RED:    red();    break;
    case GREEN:  green();  break;
    case YELLOW: yellow(); break;
  }
}

void Indicator::off() {
  digitalWrite(_pinR, HIGH);
  digitalWrite(_pinG, HIGH);
}

void Indicator::red() {
  digitalWrite(_pinR, LOW);
  digitalWrite(_pinG, HIGH);
}

void Indicator::green() {
  digitalWrite(_pinR, HIGH);
  digitalWrite(_pinG, LOW);
}

void Indicator::yellow() {
  digitalWrite(_pinR, LOW);
  digitalWrite(_pinG, LOW);
}
