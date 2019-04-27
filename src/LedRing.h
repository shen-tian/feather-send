#ifndef LEDRING_H
#define LEDRING_H

#include <FastLED.h>

#include "hal/default.h"

class LedRing
{
private:
  CRGB *_leds;
  uint8_t _dispHeading;
  uint8_t _brightness;
  long _lastTouch;

  void initRing();

public:
  LedRing();

  void init()
  {
    FastLED.addLeds<WS2812B, NEO_PIN, GRB>(_leds, NUM_LEDS);
    initRing();
  }

  void update(float targetBearing, uint8_t color);

  void dim();

  void poke();

  void goRed();
};

#endif
