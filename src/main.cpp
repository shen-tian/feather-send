#include <RH_RF95.h>

#include <Wire.h>
#include <SPI.h>
#include <Bounce2.h>

#include "hal/default.h"

#ifdef ROCKET_SCREAM
#define Serial SerialUSB
#endif

#include "State.h"
#include "LedRing.h"
#include "Imu.h"
#include "TrackerGps.h"
#include "display.h"
#include "lora.h"
#include "mac.h"

// Config
#define RF95_FREQ 915.0
#define CALLSIGN "AQUA"

// LED Ring

LedRing ledRing = LedRing();

// IMU

Imu thisImu = Imu();

// Singleton instance of the radio driver
RH_RF95 rf95 = RH_RF95(RFM95_CS, RFM95_INT);

Bounce buttonB = Bounce();
Bounce buttonC = Bounce();

// GPS
TrackerGps gps = TrackerGps();

// Timing:
#define TRANSMIT_INTERVAL 10000 // interval between sending updates
#define DISPLAY_INTERVAL 150    // interval between updating display
#define MAX_FIX_AGE 30000       // Ignore data from GPS if older

State state = State();

void updateLeds()
{

  float target = 0;

  if (state.dispMode == 0)
  {
    fix theirLoc = state.otherLocs[state.activeLoc];
    int count = state.numTrackers();

    if (count > 0)
    {
      target = initialBearing(gps.lat, theirLoc.lat, gps.lon, theirLoc.lon);
    }

    uint8_t col = (thisImu.magCal > 1) ? 127 : 0;
    ledRing.update(thisImu.heading - target, col);
  }
  else
  {
    ledRing.dim();
  }
}

// Main

void setup()
{
  state.loraFreq = RF95_FREQ;

  Serial.begin(9600);

  while (!Serial && (millis() < 3000))
  {
    delay(100);
  }

  ledRing.init();

  strcpy(state.callsign, CALLSIGN);

  pinMode(LED_PIN, OUTPUT);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(C_PIN, INPUT_PULLUP);

  initDisplay(state);
  initRadio(state, rf95);
  thisImu.init();
  //printMac();

#ifdef HAS_GPS
  gps.init();
#endif

  buttonB.attach(B_PIN);
  buttonB.interval(5);

  buttonC.attach(C_PIN);
  buttonC.interval(5); // interval in ms

  ledRing.poke();
}

void loop()
{

  updateLeds();

#ifdef HAS_GPS
  long t = millis();
  long maxGpsAge = thisImu.isStill() ? 500 : 500;
  long timeSinceFix = t - gps.fixTimestamp;

  // if (gps.hasFix() &&
  //     timeSinceFix < maxGpsAge)
  // {
  //   gps.standby();
  // }

  if (timeSinceFix > maxGpsAge)
     gps.wake();

  gps.tryRead();
#endif

  if (thisImu.exists())
  {
    thisImu.update();
    if (!thisImu.isStill())
      ledRing.poke();
  }

  buttonB.update();
  buttonC.update();

  if (buttonC.fell())
  {
    // button is pressed -- trigger action
    int count = state.numTrackers();
    if (count > 0)
    {
      state.activeLoc = (state.activeLoc + 1) % count;
    }
  }

  if (buttonB.fell())
  {
    state.dispMode = (state.dispMode + 1) % 4;
  }

  tryReceive(state, rf95);

  long sinceLastTransmit = millis() - state.lastSend;
  if (sinceLastTransmit < 0 || sinceLastTransmit > TRANSMIT_INTERVAL)
  {
    transmitData(state, rf95, gps);
    //rf95.sleep();
  }

  long sinceLastDisplayUpdate = millis() - state.lastDisplay;
  if (sinceLastDisplayUpdate < 0 || sinceLastDisplayUpdate > DISPLAY_INTERVAL)
  {
    if (millis() > 3000 || state.dispMode != 0)
      updateMainDisplay(state, thisImu, gps);
  }
}