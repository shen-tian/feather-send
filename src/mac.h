#ifndef MAC_H
#define MAC_H

#include <Wire.h>
#include <Arduino.h>

#include "hal/default.h"

#ifdef ROCKET_SCREAM
#define Serial SerialUSB
#endif

#define EUI64_CHIP_ADDRESS 0x50
#define EUI64_MAC_ADDRESS 0xF8
#define EUI64_MAC_LENGTH 0x08

void printMac();

#endif