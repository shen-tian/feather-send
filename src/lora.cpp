#include "lora.h"

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t MAGIC_NUMBER[MAGIC_NUMBER_LEN] = {0x2c, 0x0b};

char spreadFactor(uint8_t spreadFactor)
{
  switch (spreadFactor)
  {
  case 7:
    return 0x70;
  case 8:
    return 0x80;
  case 9:
    return 0x90;
  case 10:
    return 0xa0;
  case 11:
    return 0xb0;
  case 12:
    return 0xc0;
  default:
    return 0x70;
  }
}

void modemConfig(RH_RF95::ModemConfig *config, uint8_t bandwidth, uint8_t spreadFactor)
{

  config->reg_1d = 0x70 + 0x02;
  config->reg_1e = 0x70 + 0x04;
  config->reg_26 = 0x00;
}

void initRadio(State &state, RH_RF95 &rf95)
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init())
  {
    Serial.println("LoRa init: error");
    return;
  }

  Serial.println("LoRa init: successful");

  // Defaults after init are 434.0MHz, 13dBm using PA_BOOST
  // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // modulation GFSK_Rb250Fd250
  // If you are using RFM95/96/97/98 modules which uses the
  // PA_BOOST transmitter pin, then you can set transmitter
  // powers from 5 to 23 dBm:

  if (!rf95.setFrequency(state.loraFreq))
  {
    Serial.println("LoRa set freq: error");
    return;
  }

  Serial.println("LoRa set freq: successful");

  RH_RF95::ModemConfig config;

  modemConfig(&config, 125, 7);

  rf95.setModemRegisters(&config);

  rf95.setTxPower(23, false);

  Serial.println("LoRa config modem: successful");
}


void processRecv(State &state, RH_RF95 &rf95)
{

  Packet newPacket;

  memcpy(&newPacket, buf, sizeof(newPacket));

  for (int i = 0; i < MAGIC_NUMBER_LEN; i++)
  {
    if (MAGIC_NUMBER[i] != newPacket.magicNumber[i])
    {
      return;
    }
  }

  fix theirLoc;
  for (int i = 0; i < CALLSIGN_LEN; i++)
  {
    theirLoc.callsign[i] = newPacket.callsign[i];
  }

  theirLoc.timestamp = millis();
  theirLoc.rssi = rf95.lastRssi();

  theirLoc.lon = newPacket.lon;
  theirLoc.lat = newPacket.lat;
  theirLoc.isAccurate = newPacket.isAccurate;

  int slot = 0; // will displace first if all slots are full
  for (int i = 0; i < MAX_OTHER_TRACKERS; i++)
  {
    if (strlen(state.otherLocs[i].callsign) == 0 || strcmp(theirLoc.callsign, state.otherLocs[i].callsign) == 0)
    {
      slot = i;
      break;
    }
  }
  state.otherLocs[slot] = theirLoc;
}

void tryReceive(State &state, RH_RF95 &rf95)
{
  if (rf95.available())
  {
    Serial.println("Got Data");
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      Serial.println("Packed received");
      digitalWrite(LED_PIN, HIGH);
      processRecv(state, rf95);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void transmitData(State &state, RH_RF95 &rf95, TrackerGps &gps)
{
  long sinceLastFix = millis() - gps.fixTimestamp;
  if (sinceLastFix > MAX_FIX_AGE)
  {
    // GPS data is stale
    return;
  }

  Packet newPacket;

  for (int i = 0; i < MAGIC_NUMBER_LEN; i++)
  {
    newPacket.magicNumber[i] = MAGIC_NUMBER[i];
  }
  for (uint8_t i = 0; i < CALLSIGN_LEN; i++)
  {
    newPacket.callsign[i] = state.callsign[i];
  }

  newPacket.lat = gps.lat;
  newPacket.lon = gps.lon;
  newPacket.isAccurate = gps.isAccurate;

  if (abs(newPacket.lat) < 1 || abs(newPacket.lon) < 1)
  {
    // GPS data is invalid. We are on null island.
    return;
  }

  state.sending = true;
  digitalWrite(LED_PIN, HIGH);

  rf95.send((uint8_t *)&newPacket, sizeof(newPacket));
  rf95.waitPacketSent();
  digitalWrite(LED_PIN, LOW);

  state.sending = false;
  state.lastSend = millis();
}