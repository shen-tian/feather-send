#ifndef LEDRING_H
#define LEDRING_H

#include <FastLED.h>

class LedRing {
  private:
  CRGB *_leds;
  uint8_t _dispHeading;
  uint8_t _numLeds;
  float _offset;

  void initRing();

  public:
  LedRing(const uint8_t numLeds, const float offset);

  template<uint8_t DATA_PIN>
    void init(){
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(_leds, _numLeds);
    initRing();
  }

  void update(float targetBearing, uint8_t color);
  void goRed();
};

#endif
