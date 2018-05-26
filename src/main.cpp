#include <RH_RF95.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Bounce2.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// Pin layout

// Push buttons on the OLED wing
#define A_PIN 9
#define B_PIN 6
#define C_PIN 5

// Builtin LED
#define LED_PIN 13

//LoRA radio for feather m0
// INT is different for 32u4 feather
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Dimension of the display
#define LINE_PX 8
#define LINE_LEN 20

// Init display
Adafruit_SSD1306 display = Adafruit_SSD1306();

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055();

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

Bounce buttonB = Bounce();
Bounce buttonC = Bounce();

// GPS
TinyGPS gps;

// Timing:
#define TRANSMIT_INTERVAL 10000 // interval between sending updates
#define DISPLAY_INTERVAL 150    // interval between updating display
#define MAX_FIX_AGE 5000        // Ignore data from GPS if older

// State var for radio/fix
unsigned long lastSend, lastDisplay, lastFix;
bool sending = false;

#define CALLSIGN_LEN 4
#define CALLSIGN "BUTT"

#define MAX_OTHER_TRACKERS 5

typedef struct {
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

fix myLoc;
fix otherLocs[MAX_OTHER_TRACKERS];

int numSats = -1;
int activeLoc = 0;
int altitude = -1;

int dispMode = 0;

#define NUM_MODES 2

#define MAGIC_NUMBER_LEN 2

uint8_t MAGIC_NUMBER[MAGIC_NUMBER_LEN] = {0x2c, 0x0b};

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

// set > 1 to simulate more than one tracker for testing purposes
int numVirtualTrackers = 1;
int virtualTrackerNum = 0;

// Start the geometry geography stuff

// 95% error radius at HDOP=1
#define GPS_BASE_ACCURACY 6.2  // m

#define ACCURACY_THRESHOLD 30  // m

#define OTHER_LOC_STALENESS 120000

// production - burning man
// #define MAN_LAT 40786600
// #define MAN_LON -119206600
// #define PLAYA_ELEV 1190.  // m
// #define SCALE 1.

// production - afrikaburn

#define MAN_LAT -32327403
#define MAN_LON 19745329
#define PLAYA_ELEV 320.  // m
#define SCALE 1.

// testing
/*
  #define MAN_LAT 40779625
  #define MAN_LON -73965394
  #define PLAYA_ELEV 0.  // m
  #define SCALE 6.
*/

///// PLAYA COORDINATES CODE /////

#define DEG_PER_RAD (180. / 3.1415926535)
#define CLOCK_MINUTES (12 * 60)
#define METERS_PER_DEGREE (40030230. / 360.)
// Direction of north in clock units
// #define NORTH 10.5  // hours
// #define NUM_RINGS 13  // Esplanade through L
#define ESPLANADE_RADIUS (2500 * .3048)  // m
#define FIRST_BLOCK_DEPTH (440 * .3048)  // m
#define BLOCK_DEPTH (240 * .3048)  // m
// How far in from Esplanade to show distance relative to Esplanade rather than the man
#define ESPLANADE_INNER_BUFFER (250 * .3048)  // m
// Radial size on either side of 12 w/ no city streets
#define RADIAL_GAP 2.  // hours
// How far radially from edge of city to show distance relative to city streets
#define RADIAL_BUFFER .25  // hours

//// overrides for afrikaburn
#define NORTH 3.3333  // make 6ish approx line up with bearing 80 deg
#define NUM_RINGS 0  // only give distance relative to clan

// 0=man, 1=espl, 2=A, 3=B, ...
float ringRadius(int n) {
  if (n == 0) {
    return 0;
  } else if (n == 1) {
    return ESPLANADE_RADIUS;
  } else if (n == 2) {
    return ESPLANADE_RADIUS + FIRST_BLOCK_DEPTH;
  } else {
    return ESPLANADE_RADIUS + FIRST_BLOCK_DEPTH + (n - 2) * BLOCK_DEPTH;
  }
}

// Distance inward from ring 'n' to show distance relative to n vs. n-1
float ringInnerBuffer(int n) {
  if (n == 0) {
    return 0;
  } else if (n == 1) {
    return ESPLANADE_INNER_BUFFER;
  } else if (n == 2) {
    return .5 * FIRST_BLOCK_DEPTH;
  } else {
    return .5 * BLOCK_DEPTH;
  }
}

int getReferenceRing(float dist) {
  for (int n = NUM_RINGS; n > 0; n--) {
    if (ringRadius(n) - ringInnerBuffer(n) <= dist) {
      return n;
    }
  }
  return 0;
}

String getRefDisp(int n) {
  if (n == 0) {
    return ")(";
  } else if (n == 1) {
    return "Espl";
  } else {
    return String(char(int('A') + n - 2));
  }
}

String playaStr(int32_t lat, int32_t lon, bool accurate) {
  // Safe conversion to float w/o precision loss.
  float dlat = 1e-6 * (lat - MAN_LAT);
  float dlon = 1e-6 * (lon - MAN_LON);

  float m_dx = dlon * METERS_PER_DEGREE * cos(1e-6 * MAN_LAT / DEG_PER_RAD);
  float m_dy = dlat * METERS_PER_DEGREE;

  float dist = SCALE * sqrt(m_dx * m_dx + m_dy * m_dy);
  float bearing = DEG_PER_RAD * atan2(m_dx, m_dy);

  float clock_hours = (bearing / 360. * 12. + NORTH);
  int clock_minutes = (int)(clock_hours * 60 + .5);
  // Force into the range [0, CLOCK_MINUTES)
  clock_minutes = ((clock_minutes % CLOCK_MINUTES) + CLOCK_MINUTES) % CLOCK_MINUTES;

  int hour = clock_minutes / 60;
  int minute = clock_minutes % 60;
  String clock_disp = String(hour) + ":" + (minute < 10 ? "0" : "") + String(minute);

  int refRing;
  if (6 - abs(clock_minutes/60. - 6) < RADIAL_GAP - RADIAL_BUFFER) {
    refRing = 0;
  } else {
    refRing = getReferenceRing(dist);
  }
  float refDelta = dist - ringRadius(refRing);
  long refDeltaRounded = (long)(refDelta + .5);

  return clock_disp + " & " + getRefDisp(refRing) + (refDeltaRounded >= 0 ? "+" : "-") + String(refDeltaRounded < 0 ? -refDeltaRounded : refDeltaRounded) + "m" + (accurate ? "" : "-ish");
}

String fmtPlayaStr(fix* loc, char nofixstr[]) {
  if (loc->lat == 0 && loc->lon == 0) {
    return nofixstr;
  } else {
    return playaStr(loc->lat, loc->lon, loc->isAccurate);
  }
}

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
  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    if (MAGIC_NUMBER[i] != buf[i]) {
      return;
    }
  }
  fix theirLoc;
  for (int i = 0; i < CALLSIGN_LEN; i++) {
    theirLoc.callsign[i] = buf[MAGIC_NUMBER_LEN + i];
  }

  void* p = buf + MAGIC_NUMBER_LEN + CALLSIGN_LEN;
  theirLoc.lat = *(int32_t*)p;
  p = (int32_t*)p + 1;
  theirLoc.lon = *(int32_t*)p;
  p = (int32_t*)p + 1;
  theirLoc.isAccurate = *(uint8_t*)p;
  theirLoc.timestamp = millis();
  theirLoc.rssi = rf95.lastRssi();

  int slot = 0; // will displace first if all slots are full
  for (int i = 0; i < MAX_OTHER_TRACKERS; i++) {
    if (strlen(otherLocs[i].callsign) == 0 || strcmp(theirLoc.callsign, otherLocs[i].callsign) == 0) {
      slot = i;
      break;
    }
  }
  otherLocs[slot] = theirLoc;
}

void transmitData() {
  long sinceLastFix = millis() - lastFix;
  if (sinceLastFix > MAX_FIX_AGE) {
    // GPS data is stale
    return;
  }

  uint8_t len = 2 * sizeof(int32_t) + sizeof(uint8_t) + CALLSIGN_LEN + MAGIC_NUMBER_LEN + 1;
  uint8_t radiopacket[len];
  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    radiopacket[i] = MAGIC_NUMBER[i];
  }
  for (uint8_t i = 0; i < CALLSIGN_LEN; i++) {
    radiopacket[MAGIC_NUMBER_LEN + i] = myLoc.callsign[i];

    if (numVirtualTrackers > 1 && i == strlen(myLoc.callsign) - 1) {
      radiopacket[MAGIC_NUMBER_LEN + i] += virtualTrackerNum;
      virtualTrackerNum = (virtualTrackerNum + 1) % numVirtualTrackers;
    }
  }
  void* p = radiopacket + MAGIC_NUMBER_LEN + CALLSIGN_LEN;
  *(int32_t*)p = myLoc.lat;
  p = (int32_t*)p + 1;
  *(int32_t*)p = myLoc.lon;
  p = (int32_t*)p + 1;
  *(uint8_t*)p = myLoc.isAccurate;
  radiopacket[len - 1] = '\0';

  sending = true;
  rf95.send((uint8_t *)radiopacket, len);
  rf95.waitPacketSent();
  sending = false;
  lastSend = millis();
}

void setFix () {

  int32_t lat, lon;
  unsigned long age;
  gps.get_position(&lat, &lon, &age);
  if (age == TinyGPS::GPS_INVALID_AGE) {
    return;
  }
  lastFix = millis() - age;

  if (lat == TinyGPS::GPS_INVALID_ANGLE || lon == TinyGPS::GPS_INVALID_ANGLE) {
    lat = 0;
    lon = 0;
  }

  myLoc.lat = lat;
  myLoc.lon = lon;

  if (gps.hdop() == TinyGPS::GPS_INVALID_HDOP) {
    myLoc.hAcc = -1;
  } else {
    myLoc.hAcc = 1e-2 * gps.hdop() * GPS_BASE_ACCURACY;
  }
  myLoc.isAccurate = (myLoc.hAcc > 0 && myLoc.hAcc <= ACCURACY_THRESHOLD);

  numSats = gps.satellites();
  altitude = gps.altitude();
}

char timeStr[20];

void setFixTime() {
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  sprintf(timeStr, "%02d:%02d:%02d %02lds ago", hour, minute, second, age/1000);
}

void attemptUpdateFix() {
  setFixTime();
  setFix();
}

String fixAge(unsigned long timestamp) {
  long elapsed = (millis() - timestamp) / 1000;
  int n;
  char unit;
  if (elapsed < 2) {
    return "now";
  } else if (elapsed < 60) {
    n = elapsed;
    unit = 's';
  } else if (elapsed < 3600) {
    n = elapsed / 60;
    unit = 'm';
  } else {
    n = elapsed / 3600;
    unit = 'h';
  }
  return String(n) + String(unit) + " ago";
}

int getNumTrackers() {
  int i;
  for (i = 0; i < MAX_OTHER_TRACKERS; i++) {
    if (strlen(otherLocs[i].callsign) == 0) {
      break;
    }
  }
  return i;
}

String getCallsigns() {
  int count = getNumTrackers();
  if (count == 0) {
    return "all alone";
  }
  String s = "";
  for (int i = 0; i < count; i++) {
    int slot = (activeLoc + i) % count;
    if (i == 0) {
      s.concat(">");
    }
    String cs = String(otherLocs[slot].callsign);
    if (millis() - otherLocs[slot].timestamp > OTHER_LOC_STALENESS) {
      cs.toLowerCase();
    } else {
      cs.toUpperCase();
    }
    s.concat(cs);
    //if (i == 0) {
    //  s.concat("]");
    //}
    s.concat(" ");
    if (s.length() > LINE_LEN) {
      break;
    }
  }
  return s.substring(0, LINE_LEN);
}

// Distance/bearing Math from here:
// https://www.movable-type.co.uk/scripts/latlong.html

float distanceFromLoc(int lat0, int lat1, int lon0, int lon1)
{
  int R = 6371e3; // metres
  float t0 = PI * lat0 / (180.0 * 1e6);
  float t1 = PI * lat1 / (180.0 * 1e6);
  float dt = PI * (lat1 - lat0) / (180.0 * 1e6);
  float dl = PI * (lon1 - lon0) / (180.0 * 1e6);

  float a = sin(dt/2) * sin(dt/2) +
    cos(t0) * cos(t1) *
    sin(dl/2) * sin(dl/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));

  return R * c;
}

float initialBearing(int lat0, int lat1, int lon0, int lon1)
{
  float t0 = PI * lat0 / (180.0 * 1e6);
  float t1 = PI * lat1 / (180.0 * 1e6);
  float l0 = PI * lon0 / (180.0 * 1e6);
  float l1 = PI * lon1 / (180.0 * 1e6);

  float y = sin(l1-l0) * cos(t1);
  float x = cos(t0) * sin(t1) -
    sin(t0) * cos(t1) * cos(l1 - l0);

  if (x == 0 && y == 0)
    return -1;
  else
    return atan2(y, x) * 180 / PI;
}

String locationString(fix* loc, char nofixstr[])
{
  char line[20];

  float distance = distanceFromLoc(loc->lat, myLoc.lat, loc->lon, myLoc.lon);

  float bearing = initialBearing(myLoc.lat, loc->lat, myLoc.lon, loc->lon);

  sprintf(line, "%.1fm away, %.0fdeg", distance, bearing);

  return line;
}

imu::Vector<3> euler, mag, gyro;
int8_t temperature;
int32_t heading;
uint8_t sysCal, gyroCal, accCal, magCal;


#define MAG_DEC -25
#define SENSOR_HEADING 180

void updateDirection(){

    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    temperature = bno.getTemp();

    heading = euler.x() + MAG_DEC + SENSOR_HEADING;
    if (heading > 360)
      heading -= 360;

    bno.getCalibration(&sysCal, &gyroCal, &accCal, &magCal);
}


void showHeading(fix* loc){
  float bearing = initialBearing(myLoc.lat, loc->lat, myLoc.lon, loc->lon);

  float diff = heading - bearing;
  while (diff > 180)
    diff -= 360;
  while (diff < -180)
    diff += 180;

  int8_t offset = diff / 360.0 * 127;


  display.fillRect(64 - offset, 3*LINE_PX, 1, 8, 1);
  display.fillRect(63, 3*LINE_PX,3,2,1);
}


void updateDisplay() {
  fix theirLoc = otherLocs[activeLoc];
  int count = getNumTrackers();

  display.clearDisplay();

  display.setCursor(0, 0);
  display.println(getCallsigns());
  if (count > 0) {
    char msg[10];
    strcpy(msg,"");
    display.println(locationString(&theirLoc, msg));
    display.println(fixAge(theirLoc.timestamp));
    display.setCursor(60, 2*LINE_PX);
    display.println(String(theirLoc.rssi) + "db");
    showHeading(&theirLoc);
  }
  display.setCursor(0, 3*LINE_PX);
  //char msg[10];
  //strcpy(msg, "and lost");

  String fixStatus = "";
  long sinceLastFix = millis() - lastFix;
  long sinceLastSend = millis() - lastSend;
  if (sinceLastFix > MAX_FIX_AGE) {
    // GPS data is stale
    fixStatus = "!";
  } else if (sending || (sinceLastSend >= 0 && sinceLastSend < 400)) {
    fixStatus = ".";
  }
  display.setCursor(120, 3*LINE_PX);
  display.println(fixStatus);

  display.display();

  lastDisplay = millis();
}

const uint8_t satIcon[] PROGMEM = {0x00, 0x26, 0x74, 0x38, 0x14, 0x68, 0x40, 0x00};
const uint8_t altIcon[] PROGMEM = {0x00, 0x00, 0x18, 0x1c, 0x7e, 0x7e, 0xff, 0x00};

void updateGpsDisplay(){
  display.clearDisplay();
  char buff[20];

  display.setCursor(0, 0);
  display.println(timeStr);

  sprintf(buff, "lat:%11.6f   %2d", myLoc.lat / 1e6, numSats);
  display.println(buff);

  sprintf(buff, "lon:%11.6f", myLoc.lon / 1e6);
  display.println(buff);

  sprintf(buff, "acc:%4.0fm alt:%4dm", myLoc.hAcc, altitude / 100);
  display.println(buff);

  display.drawBitmap(17 * 6, 1 * LINE_PX, satIcon, 8, 8, 1);

  display.display();
  lastDisplay = millis();
}

void updateImuDisplay(){
 display.clearDisplay();
  char buff[20];

  sprintf(buff, "br %3ld tp %2d S%dG%dA%dM%d", heading, temperature, sysCal, gyroCal, accCal, magCal);
  display.setCursor(0, 0);
  display.println(buff);

  sprintf(buff, "eu x%5.0fy%5.0fz%5.0f", euler.x(), euler.y(), euler.z());
  display.println(buff);

  sprintf(buff, "ma x%5.1fy%5.1fz%5.1f", mag.x(), mag.y(), mag.z());
  display.println(buff);

  sprintf(buff, "gy x%5.1fy%5.1fz%5.1f", gyro.x(), gyro.y(), gyro.z());
  display.println(buff);

  display.display();
  lastDisplay = millis();
}

void updateSystemDisplay(){
  display.clearDisplay();
  char buff[20];

  float measuredvbat = analogRead(A7);
  measuredvbat *= (2 * 3.3 / 1024);
  sprintf(buff, "VBatt: %.2fV", measuredvbat);
  display.setCursor(0, 0);
  display.println(buff);

  sprintf(buff, "Uptime: %ldms", millis());
  display.println(buff);

  display.display();
  lastDisplay = millis();
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
    say("x0", "", "", "");
    while (1);
  }

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
  rf95.setTxPower(23, false);
}

void initDisplay(){
  // initialize with the I2C addr 0x3C (for the 128x32)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  say("hello " + String(myLoc.callsign) + ".", "", "", "");
  delay(3000);
  display.clearDisplay();
}

void initImu(){
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

// Main

void setup() {
  strcpy(myLoc.callsign, CALLSIGN);

  pinMode(LED_PIN, OUTPUT);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(A_PIN, INPUT_PULLUP);

  initDisplay();
  initRadio();
  initImu();

  Serial1.begin(9600);

  buttonB.attach(B_PIN);
  buttonB.interval(5);

  buttonC.attach(C_PIN);
  buttonC.interval(5); // interval in ms
}

void loop() {
  updateDirection();

  buttonB.update();
  buttonC.update();

  if (buttonC.fell()) {
    // button is pressed -- trigger action
    int count = getNumTrackers();
    if (count > 0) {
      activeLoc = (activeLoc + 1) % count;
    }
  }

  if (buttonB.fell()) {
    dispMode = (dispMode + 1) % 4;
  }


  if (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) { // Did a new valid sentence come in?
      attemptUpdateFix();
    }
  }

  if (rf95.available()) {
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(LED_PIN, LOW);
      processRecv();
    }
  }

  long sinceLastTransmit = millis() - lastSend;
  if (sinceLastTransmit < 0 || sinceLastTransmit > TRANSMIT_INTERVAL) {
    transmitData();
  }

  long sinceLastDisplayUpdate = millis() - lastDisplay;
  if (sinceLastDisplayUpdate < 0 || sinceLastDisplayUpdate > DISPLAY_INTERVAL) {
    switch(dispMode) {
    case 0: updateDisplay();
      break;
    case 1: updateGpsDisplay();
      break;
    case 2: updateImuDisplay();
      break;
    case 3: updateSystemDisplay();
      break;
    }
  }
}
