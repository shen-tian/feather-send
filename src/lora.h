#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include <RH_RF95.h>

#include "hal/default.h"

#ifdef ROCKET_SCREAM
#define Serial SerialUSB
#endif

#include "State.h"
#include "TrackerGps.h"

#define MAGIC_NUMBER_LEN 2
#define MAX_FIX_AGE 30000

void initRadio(State &state, RH_RF95 &rf95);

void tryReceive(State &state, RH_RF95 &rf95);

void transmitData(State &state, RH_RF95 &rf95, TrackerGps &gps);

#endif