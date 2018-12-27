#include "display.h"

const char* timeSince(unsigned long timestamp) {
  long elapsed = (millis() - timestamp) / 1000;
  int n;
  char unit;

  static char buff[20];

  if (elapsed < 2) {
      strcpy(buff, "now");
      return buff;
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

  sprintf(buff, "%d%c", n, unit);
  return buff;
}

void updateSystemDisplay(State &state, Adafruit_SSD1306 &display){
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

void updateGpsDisplay(State &state, Adafruit_SSD1306 &display, TrackerGps &gps){
  display.clearDisplay();
  char buff[50];

  display.setCursor(0, 0);
  //sprintf(buff, "%02d:%02d:%02d %02lds ago", gps.hour, gps.minute, gps.second, (millis() - gps.fixTimestamp)/1000);
  
  int age = (millis() - gps.fixTimestamp) / 1000;
  byte second = gps.second + age;
  byte hour = gps.hour;
  byte minute = gps.minute;

  while (second >= 60) {
    second -= 60;
    minute++;
  }

  while (minute >= 60) {
    minute -= 60;
    hour++;
  }

  while (hour >= 24) {
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

  sprintf(buff, "acc:%4.0fm alt:%4dm", gps.hAccuracy, gps.altitude / 100);
  display.println(buff);

  display.drawBitmap(17 * 6, 1 * LINE_PX, satIcon, 8, 8, 1);

  display.display();
  state.lastDisplay = millis();
}

void updateImuDisplay(State &state, Adafruit_SSD1306 &display, Imu &imu){
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