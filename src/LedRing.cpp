#include "LedRing.h"

LedRing::LedRing (const uint8_t numLeds, const float offset) {
  _dispHeading = 0;
  _offset = 0;
  _numLeds = numLeds;
  _leds = new CRGB[numLeds];
}

void LedRing::initRing(){
  for (int i = 0; i < _numLeds; i++)
    _leds[i] = CRGB::Black;
  FastLED.show();
}

void LedRing::update(float targetBearing, uint8_t color){
  uint8_t target = (uint8_t)((targetBearing + _offset) * 0.7111);

  int8_t gap = target - _dispHeading;

  _dispHeading += (int8_t)(gap * .3);

  for (int i = 0; i < _numLeds; i++)
  {
    uint8_t pos = (_numLeds - i) * 255 / _numLeds;
    uint8_t distance = abs(_dispHeading - pos);
    if (distance > 127)
      distance = 255 - distance;
    uint8_t val = max(255 - distance * 8, 0);

    uint8_t sat = 255 * .6;

    _leds[i] = CHSV(color, sat, val);
  }
  FastLED.show();
}

void LedRing::goRed(){
  for (int i = 0; i < _numLeds; i++)
    _leds[i] = CRGB::Red;

  FastLED.show();
}
