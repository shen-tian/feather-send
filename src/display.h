#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_SSD1306.h>

#include "State.h"
#include "hal/default.h"
#include "FreeMemory.h"
#include "TrackerGps.h"
#include "Imu.h"

#define LINE_PX 8

const uint8_t satIcon[] PROGMEM = {0x00, 0x26, 0x74, 0x38, 0x14, 0x68, 0x40, 0x00};

void updateSystemDisplay(State &state, Adafruit_SSD1306 &display);

void updateGpsDisplay(State &state, Adafruit_SSD1306 &display, TrackerGps &gps);

void updateImuDisplay(State &state, Adafruit_SSD1306 &display, Imu &imu);

#endif