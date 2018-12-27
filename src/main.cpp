#include <RH_RF95.h>
#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
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

// Push buttons on the OLED wing
#define A_PIN 9
#define B_PIN 6
#define C_PIN 5

// Builtin LED
#define LED_PIN 13

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Dimension of the display
#define LINE_PX 8
#define LINE_LEN 20

// Init display
Adafruit_SSD1306 display = Adafruit_SSD1306(128, SCREEN_H);

// LEDs
#define NUM_LEDS 12
#define LED_OFFSET 285
#define NEO_PIN 10

LedRing ledRing = LedRing(NUM_LEDS, LED_OFFSET);

// IMU

#define MAG_DEC -25
#define SENSOR_HEADING 90

Imu thisImu = Imu(MAG_DEC, SENSOR_HEADING);

// Singleton instance of the radio driver
RH_RF95 rf95 = RH_RF95(RFM95_CS, RFM95_INT);

Bounce buttonB = Bounce();
Bounce buttonC = Bounce();

// GPS
TrackerGps gps = TrackerGps();

// Timing:
#define TRANSMIT_INTERVAL 10000 // interval between sending updates
#define DISPLAY_INTERVAL 150    // interval between updating display
#define MAX_FIX_AGE 30000        // Ignore data from GPS if older

#define CALLSIGN_LEN 4
#define CALLSIGN "DIGS"

State state = State();

#define MAGIC_NUMBER_LEN 2

uint8_t MAGIC_NUMBER[MAGIC_NUMBER_LEN] = {0x2c, 0x0b};

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// 4-line display
void say(String s, String t, String u, String v) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(s);
  display.println(t);
  display.println(u);
  display.println(v);
  display.display();
}

void processRecv() {

  Packet newPacket;

  memcpy(&newPacket, buf, sizeof(newPacket));

  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    if (MAGIC_NUMBER[i] != newPacket.magicNumber[i]) {
      return;
    }
  }

  fix theirLoc;
  for (int i = 0; i < CALLSIGN_LEN; i++) {
    theirLoc.callsign[i] = newPacket.callsign[i];
  }

  theirLoc.timestamp = millis();
  theirLoc.rssi = rf95.lastRssi();

  theirLoc.lon = newPacket.lon;
  theirLoc.lat = newPacket.lat;
  theirLoc.isAccurate = newPacket.isAccurate;

  int slot = 0; // will displace first if all slots are full
  for (int i = 0; i < MAX_OTHER_TRACKERS; i++) {
    if (strlen(state.otherLocs[i].callsign) == 0 || strcmp(theirLoc.callsign, state.otherLocs[i].callsign) == 0) {
      slot = i;
      break;
    }
  }
  state.otherLocs[slot] = theirLoc;
}

void transmitData() {
  long sinceLastFix = millis() - gps.fixTimestamp;
  if (sinceLastFix > MAX_FIX_AGE) {
    // GPS data is stale
    return;
  }

  Packet newPacket;

  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    newPacket.magicNumber[i] = MAGIC_NUMBER[i];
  }
  for (uint8_t i = 0; i < CALLSIGN_LEN; i++) {
    newPacket.callsign[i] = state.myLoc.callsign[i];
  }

  newPacket.lat = gps.lat;
  newPacket.lon = gps.lon;
  newPacket.isAccurate = gps.isAccurate;

  state.sending = true;
  digitalWrite(LED_PIN, HIGH);
   
  rf95.send((uint8_t*)&newPacket, sizeof(newPacket));
  rf95.waitPacketSent();
  digitalWrite(LED_PIN, LOW);
   
  state.sending = false;
  state.lastSend = millis();
}

// void showHeading(fix* loc){
//   float bearing = initialBearing(gps.lat, loc->lat, gps.lon, loc->lon);

//   float diff = thisImu.heading - bearing;
//   while (diff > 180)
//     diff -= 360;
//   while (diff < -180)
//     diff += 180;

//   int8_t offset = diff / 360.0 * 127;

//   display.fillRect(64 - offset, 3*LINE_PX, 1, 8, 1);
//   display.fillRect(63, 3*LINE_PX,3,2,1);
// }

void updateLeds(){

  float target = 0;

  if (state.dispMode == 0) {
     fix theirLoc = state.otherLocs[state.activeLoc];
     int count = state.numTrackers();

     if (count > 0) {
       target = initialBearing(gps.lat, theirLoc.lat, gps.lon, theirLoc.lon);
     }
  }

  uint8_t col = (thisImu.magCal > 1) ? 127 : 0;

  if (state.dispMode == 0)
    col += 64;

  ledRing.update(thisImu.heading - target, col);
}

char spreadFactor(uint8_t spreadFactor){
  switch(spreadFactor) {
    case 7: return 0x70;
    case 8: return 0x80;
    case 9: return 0x90;
    case 10: return 0xa0;
    case 11: return 0xb0;
    case 12: return 0xc0;
    default: return 0x70;
  }
}

void modemConfig(RH_RF95::ModemConfig* config, uint8_t bandwidth, uint8_t spreadFactor){

  config->reg_1d = 0x70 + 0x02;
  config->reg_1e = 0x70 + 0x04;
  config->reg_26 = 0x00;
}

void initRadio(){
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    say("LoRa init", "", "", "");
    while (1);
  }

  Serial.println("LoRa init: successful");

  // Defaults after init are 434.0MHz, 13dBm using PA_BOOST
  // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // modulation GFSK_Rb250Fd250
  // If you are using RFM95/96/97/98 modules which uses the
  // PA_BOOST transmitter pin, then you can set transmitter
  // powers from 5 to 23 dBm:

  if (!rf95.setFrequency(RF95_FREQ)) {
    say("x1", "", "", "");
    while (1);
  }

  Serial.println("LoRa set freq: successful");

  RH_RF95::ModemConfig config;

  modemConfig(&config, 125, 7);

  rf95.setModemRegisters(&config);

  rf95.setTxPower(23, false);

  Serial.println("LoRa config modem: successful");
}

void initDisplay(){
  
  Serial.println("Display: init");
  // initialize with the I2C addr 0x3C (for the 128x32)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  say("hello " + String(state.myLoc.callsign) + ".", "", "", "");
  delay(3000);
  display.clearDisplay();
}

// Main

void setup() {

  state.loraFreq = RF95_FREQ;

  Serial.begin(9600);

  while (!Serial && (millis() < 3000)) {
    delay(100);
  }

  ledRing.init<NEO_PIN>();

  strcpy(state.myLoc.callsign, CALLSIGN);

  pinMode(LED_PIN, OUTPUT);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(C_PIN, INPUT_PULLUP);

  initDisplay();
  initRadio();

  thisImu.init();

#ifdef HAS_GPS
  gps.init();
#endif

  buttonB.attach(B_PIN);
  buttonB.interval(5);

  buttonC.attach(C_PIN);
  buttonC.interval(5); // interval in ms

  ledRing.poke();
}

void loop() {

  updateLeds();

#ifdef HAS_GPS
  long t = millis();
  long maxGpsAge = thisImu.isStill() ? 30000 : 500;
  long timeSinceFix = t - gps.fixTimestamp;

  if (gps.hasFix() &&
      timeSinceFix < maxGpsAge) {
    gps.standby();
  }

  if (timeSinceFix > maxGpsAge)
    gps.wake();

  gps.tryRead();
#endif

  if (thisImu.exists()) {
    thisImu.update();
    if (!thisImu.isStill())
      ledRing.poke();
  }

  buttonB.update();
  buttonC.update();

  if (buttonC.fell()) {
    // button is pressed -- trigger action
    int count = state.numTrackers();
    if (count > 0) {
      state.activeLoc = (state.activeLoc + 1) % count;
    }
  }

  if (buttonB.fell()) {
    state.dispMode = (state.dispMode + 1) % 4;
  }

  if (rf95.available()) {
    Serial.println("Got Data");
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial.println("Packed received");
      digitalWrite(LED_PIN, HIGH);
      processRecv();
      digitalWrite(LED_PIN, LOW);
    }
  }

  long sinceLastTransmit = millis() - state.lastSend;
  if (sinceLastTransmit < 0 || sinceLastTransmit > TRANSMIT_INTERVAL) {
    transmitData();
    //rf95.sleep();
  }

  long sinceLastDisplayUpdate = millis() - state.lastDisplay;
  if (sinceLastDisplayUpdate < 0 || sinceLastDisplayUpdate > DISPLAY_INTERVAL) {
    switch(state.dispMode) {
    case 0: updateMainDisplay(state, display, gps);
      break;
    case 1: updateGpsDisplay(state, display, gps);
      break;
    case 2: updateImuDisplay(state, display, thisImu);
      break;
    case 3: updateSystemDisplay(state, display);
      break;
    }
  }
}