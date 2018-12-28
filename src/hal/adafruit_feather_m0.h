#ifndef HAL_H
#define HAL_H

// Adafruit Feather M0

#define VBAT_PIN A7
#define VBAT_RATIO (6.6 / 1023.0)

// LoRa

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

#define SCREEN_H 32

#define HAS_GPS
#define HAS_IMU

// Builtin LED
#define LED_PIN 13

#endif