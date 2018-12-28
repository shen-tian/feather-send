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

// Builtin LED
#define LED_PIN 13

#endif