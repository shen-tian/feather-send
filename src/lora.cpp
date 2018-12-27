#include "lora.h"

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