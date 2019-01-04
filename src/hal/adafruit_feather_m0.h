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

// LEDs
#define NUM_LEDS 12
#define LED_OFFSET 285
#define NEO_PIN 10

// Push buttons on the OLED wing
#define A_PIN 9
#define B_PIN 6
#define C_PIN 5

// Magnetic sensor

#define MAG_DEC -25
#define SENSOR_HEADING 90

#endif