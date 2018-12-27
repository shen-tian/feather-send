#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include "State.h"
#include "hal/default.h"
#include "FreeMemory.h"
#include "TrackerGps.h"
#include "Imu.h"

#define LINE_PX 8
#define LINE_LEN 20

// should be a shared config
#define MAX_FIX_AGE 30000
#define OTHER_LOC_STALENESS 120000

const uint8_t satIcon[] PROGMEM = {0x00, 0x26, 0x74, 0x38, 0x14, 0x68, 0x40, 0x00};
const uint8_t altIcon[] PROGMEM = {0x00, 0x00, 0x18, 0x1c, 0x7e, 0x7e, 0xff, 0x00};

void initDisplay(State &state);

void updateMainDisplay(State &state, Imu &imu, TrackerGps &gps);

float initialBearing(int lat0, int lat1, int lon0, int lon1);

#endif