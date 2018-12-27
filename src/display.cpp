#include "display.h"

void initDisplay(State &state, Adafruit_SSD1306 &display)
{

  Serial.println("Display: init");
  // initialize with the I2C addr 0x3C (for the 128x32)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("hello " + String(state.callsign) + ".");
  display.display();
}

const char *timeSince(unsigned long timestamp)
{
  long elapsed = (millis() - timestamp) / 1000;
  int n;
  char unit;

  static char buff[20];

  if (elapsed < 2)
  {
    strcpy(buff, "now");
    return buff;
  }
  else if (elapsed < 60)
  {
    n = elapsed;
    unit = 's';
  }
  else if (elapsed < 3600)
  {
    n = elapsed / 60;
    unit = 'm';
  }
  else
  {
    n = elapsed / 3600;
    unit = 'h';
  }

  sprintf(buff, "%d%c", n, unit);
  return buff;
}

void updateSystemDisplay(State &state, Adafruit_SSD1306 &display)
{
  display.clearDisplay();

  char buff[50];

  float measuredvbat = 0;

  for (int i = 0; i < 8; i++)
    measuredvbat += analogRead(VBAT_PIN);

  measuredvbat /= 8;
  measuredvbat *= VBAT_RATIO;

  sprintf(buff, "VBatt: %.2fV", measuredvbat);
  display.setCursor(0, 0);
  display.println(buff);

  sprintf(buff, "Uptime: %s", timeSince(0));
  display.println(buff);

  sprintf(buff, "Free mem: %dkb", freeMemory());
  display.println(buff);

  sprintf(buff, "Freq: %.2fMhz", state.loraFreq);
  display.println(buff);

  display.display();
  state.lastDisplay = millis();
}

void updateGpsDisplay(State &state, Adafruit_SSD1306 &display, TrackerGps &gps)
{
  display.clearDisplay();
  char buff[50];

  display.setCursor(0, 0);
  //sprintf(buff, "%02d:%02d:%02d %02lds ago", gps.hour, gps.minute, gps.second, (millis() - gps.fixTimestamp)/1000);

  int age = (millis() - gps.fixTimestamp) / 1000;
  byte second = gps.second + age;
  byte hour = gps.hour;
  byte minute = gps.minute;

  while (second >= 60)
  {
    second -= 60;
    minute++;
  }

  while (minute >= 60)
  {
    minute -= 60;
    hour++;
  }

  while (hour >= 24)
  {
    hour--;
  }

  sprintf(buff, "%02d:%02d:%02d %d", hour, minute, second, age);

  display.println(buff);
  if (gps.isAwake())
    display.fillCircle(123, 3, 2, 1);

  sprintf(buff, "lat:%11.6f   %2d", gps.lat / 1e6, gps.numSats);
  display.println(buff);

  sprintf(buff, "lon:%11.6f", gps.lon / 1e6);
  display.println(buff);

  sprintf(buff, "acc:%4.0fm alt:%4dm", gps.hAccuracy, (int)(gps.altitude / 100));
  display.println(buff);

  display.drawBitmap(17 * 6, 1 * LINE_PX, satIcon, 8, 8, 1);

  display.display();
  state.lastDisplay = millis();
}

void updateImuDisplay(State &state, Adafruit_SSD1306 &display, Imu &imu)
{
  display.clearDisplay();
  char buff[20];

  sprintf(buff, "br %3ld tp %2d S%dG%dA%dM%d", imu.heading, imu.temperature, imu.sysCal, imu.gyroCal, imu.accCal, imu.magCal);
  display.setCursor(0, 0);
  display.println(buff);

  sprintf(buff, "eu x%5.0fy%5.0fz%5.0f", imu.euler.x(), imu.euler.y(), imu.euler.z());
  display.println(buff);

  sprintf(buff, "ma x%5.1fy%5.1fz%5.1f", imu.mag.x(), imu.mag.y(), imu.mag.z());
  display.println(buff);

  sprintf(buff, "gy x%5.1fy%5.1fz%5.1f", imu.gyro.x(), imu.gyro.y(), imu.gyro.z());
  display.println(buff);

  display.display();
  state.lastDisplay = millis();
}

String getCallsigns(State &state)
{
  int count = state.numTrackers();
  if (count == 0)
  {
    return "all alone";
  }
  String s = "";
  for (int i = 0; i < count; i++)
  {
    int slot = (state.activeLoc + i) % count;
    if (i == 0)
    {
      s.concat(">");
    }
    String cs = String(state.otherLocs[slot].callsign);
    if (millis() - state.otherLocs[slot].timestamp > OTHER_LOC_STALENESS)
    {
      cs.toLowerCase();
    }
    else
    {
      cs.toUpperCase();
    }
    s.concat(cs);
    //if (i == 0) {
    //  s.concat("]");
    //}
    s.concat(" ");
    if (s.length() > LINE_LEN)
    {
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

  float a = sin(dt / 2) * sin(dt / 2) +
            cos(t0) * cos(t1) *
                sin(dl / 2) * sin(dl / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}

float initialBearing(int lat0, int lat1, int lon0, int lon1)
{
  float t0 = PI * lat0 / (180.0 * 1e6);
  float t1 = PI * lat1 / (180.0 * 1e6);
  float l0 = PI * lon0 / (180.0 * 1e6);
  float l1 = PI * lon1 / (180.0 * 1e6);

  float y = sin(l1 - l0) * cos(t1);
  float x = cos(t0) * sin(t1) -
            sin(t0) * cos(t1) * cos(l1 - l0);

  if (x == 0 && y == 0)
    return -1;
  else
    return atan2(y, x) * 180 / PI;
}

String locationString(fix *loc, char nofixstr[], TrackerGps gps)
{
  char line[20];

  float distance = distanceFromLoc(loc->lat, gps.lat, loc->lon, gps.lon);

  float bearing = initialBearing(gps.lat, loc->lat, gps.lon, loc->lon);

  if (distance < 1000)
  {
    sprintf(line, "%.0fm away, %.0fdeg", distance, bearing);
  }
  else
  {
    sprintf(line, "%.1fkm away, %.0fdeg ", distance / 1000, bearing);
  }

  return line;
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

void updateMainDisplay(State &state, Adafruit_SSD1306 &display, TrackerGps &gps)
{
  fix theirLoc = state.otherLocs[state.activeLoc];
  int count = state.numTrackers();

  display.clearDisplay();

  display.setCursor(0, 0);
  display.println(getCallsigns(state));
  if (count > 0)
  {
    char msg[10];
    strcpy(msg, "");
    display.println(locationString(&theirLoc, msg, gps));
    display.println(timeSince(theirLoc.timestamp));
    display.setCursor(60, 2 * LINE_PX);
    display.println(String(theirLoc.rssi) + "db");
    //showHeading(&theirLoc);
  }
  display.setCursor(0, 3 * LINE_PX);
  //char msg[10];
  //strcpy(msg, "and lost");

  String fixStatus = "";
  long sinceLastFix = millis() - gps.fixTimestamp;
  long sinceLastSend = millis() - state.lastSend;
  if (sinceLastFix > MAX_FIX_AGE)
  {
    // GPS data is stale
    fixStatus = "!";
  }
  else if (state.sending || (sinceLastSend >= 0 && sinceLastSend < 400))
  {
    fixStatus = ".";
  }
  display.setCursor(120, 3 * LINE_PX);
  display.println(fixStatus);

  display.display();

  state.lastDisplay = millis();
}

void updateMainDisplay(State &state, Adafruit_SSD1306 &display, Imu &imu, TrackerGps &gps)
{
  switch (state.dispMode)
  {
  case 0:
    updateMainDisplay(state, display, gps);
    break;
  case 1:
    updateGpsDisplay(state, display, gps);
    break;
  case 2:
    updateImuDisplay(state, display, imu);
    break;
  case 3:
    updateSystemDisplay(state, display);
    break;
  }
}