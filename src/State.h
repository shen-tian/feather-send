#ifndef STATE_H
#define STATE_H

#include <stdint.h>

#define CALLSIGN_LEN 4
#define MAX_OTHER_TRACKERS 5

typedef struct fix {
  // final null will never be overwritten
  char callsign[CALLSIGN_LEN + 1] = {0x0, 0x0, 0x0, 0x0, 0x0};
  unsigned long timestamp;
  // lat/lon are stored as signed 32-bit ints as millionths
  // of a degree (-123.45678 => -123,456,780)
  
  int32_t lat;
  int32_t lon;
  float elev;  // unused
  float hAcc;
  bool isAccurate;
  int rssi;
} fix;

// Don't pack this, for easier marshalling
#pragma pack(1)

typedef struct Packet {
  uint8_t magicNumber[2];
  char callsign[4];
  int32_t lat;
  int32_t lon;

  char isAccurate;
} Packet;

#pragma pack()

class State {

  public:

  char callsign[4];
  fix otherLocs[MAX_OTHER_TRACKERS];
  int activeLoc = 0;
  int dispMode = 0;
  float loraFreq;
  unsigned long lastSend, lastDisplay;
  bool sending = false;

  State() {}

  int numTrackers() {
    int i;
    for (i = 0; i < MAX_OTHER_TRACKERS; i++) {
      if (strlen(otherLocs[i].callsign) == 0) {
        break;  
      }
    }
    return i;
  }
};

#endif