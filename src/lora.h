#ifndef LORA_H
#define LORA_H

#include <stdint.h>
#include <RH_RF95.h>

void modemConfig(RH_RF95::ModemConfig *config, uint8_t bandwidth, uint8_t spreadFactor);

#endif