#ifndef TRACKERGPS_H
#define TRACKERGPS_H

#include <TinyGPS.h>

// 95% error radius at HDOP=1
#define GPS_BASE_ACCURACY 6.2  // m
#define ACCURACY_THRESHOLD 30  // m

class TrackerGps {
 private:
  bool _standby = false;
  TinyGPS _parser;

 public:

  int32_t lat, lon;
  unsigned long age;

  int8_t numSats;

  int32_t altitude;
  float hAccuracy;

  bool isAccurate;

  TrackerGps() {};

  void init();

  void standby();

  void wake();

  bool isAwake();

  void tryRead();
};

#endif
