#ifndef HAL_H
#define HAL_H

// Arduino Zero (Rocket Scream HAL)

#define ROCKET_SCREAM

// Battery Monitoring

#define VBAT_PIN A5
#define VBAT_RATIO (4.3 / 1023.0)

// LoRa

#define RFM95_CS 5
#define RFM95_INT 2
#define RFM95_RST 4

// Scream

#define SCREEN_H 64

#define HAS_GPS

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