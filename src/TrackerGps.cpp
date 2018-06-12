#include "TrackerGps.h"

void TrackerGps::init() {
  Serial1.begin(9600);
  Serial1.println("$PMTK220,1000*1F");
}

void TrackerGps::standby() {
  if (!_standby){
    Serial1.println("$PMTK161,0*28");
    _standby = true;
  }
}

void TrackerGps::wake() {
  if (_standby){
    Serial1.println("");
    _standby = false;
  }
}

bool TrackerGps::isAwake() {
  return !_standby;
}

void TrackerGps::tryRead() {
  if (!_standby && Serial1.available()) {
    char c = Serial1.read();
    if (_parser.encode(c)) {
      int32_t newLat, newLon;
      unsigned long newAge;

      _parser.get_position(&newLat, &newLon, &newAge);
      if (newAge != TinyGPS::GPS_INVALID_AGE &&
          newLat != TinyGPS::GPS_INVALID_ANGLE &&
          newLon != TinyGPS::GPS_INVALID_ANGLE) {
        age = newAge;
        lat = newLat;
        lon = newLon;
      }

      numSats = _parser.satellites();
      altitude = _parser.altitude();

      float hdop = _parser.hdop();

      if (hdop == TinyGPS::GPS_INVALID_HDOP) {
        hAccuracy = -1;
      } else {
        hAccuracy = 1e-2 * hdop * GPS_BASE_ACCURACY;
      }

      isAccurate = (hAccuracy > 0 && hAccuracy <= ACCURACY_THRESHOLD);
    }
  }
}
