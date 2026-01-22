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
    case RED:    red();    break;
    case GREEN:  green();  break;
    case BLUE:   blue();   break;
    case YELLOW: yellow(); break;
  }
}

void Indicator::off() {
  digitalWrite(_pinR, HIGH);
  digitalWrite(_pinG, HIGH);
  digitalWrite(_pinB, HIGH);
}

void Indicator::red() {
  digitalWrite(_pinR, LOW);
  digitalWrite(_pinG, HIGH);
  digitalWrite(_pinB, HIGH);
}

void Indicator::green() {
    digitalWrite(_pinR, HIGH);
    digitalWrite(_pinG, LOW);
    digitalWrite(_pinB, HIGH);
}

void Indicator::yellow() {
    digitalWrite(_pinR, LOW);
    digitalWrite(_pinG, LOW);
    digitalWrite(_pinB, HIGH);
}

void Indicator::blue() {
    digitalWrite(_pinR, HIGH);
    digitalWrite(_pinG, HIGH);
    digitalWrite(_pinB, LOW);
}