#ifndef HAL_H
#define HAL_H

// This file is just here to make VSCode happy

// Battery monitoring

#define VBAT_PIN 0
#define VBAT_RATIO 0.0

// LoRa

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

// Screen

#define SCREEN_H 32

// Component toggles

#define HAS_GPS
#define HAS_IMU

// Builtin LED
#define LED_PIN 13

// LEDs
#define NUM_LEDS 12
#define LED_OFFSET 270
#define NEO_PIN 10

// Push buttons on the OLED wing
#define A_PIN 9
#define B_PIN 6
#define C_PIN 5

// Magnetic sensor

#define MAG_DEC -25
#define SENSOR_HEADING -90

#endif