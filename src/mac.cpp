#include "mac.h"

void setDevEui(unsigned char* buf)
{
  Wire.begin();
  Wire.beginTransmission(EUI64_CHIP_ADDRESS);
  Wire.write(EUI64_MAC_ADDRESS);
  Wire.endTransmission();
  Wire.requestFrom(EUI64_CHIP_ADDRESS, EUI64_MAC_LENGTH);

  // Format needs to be little endian (LSB...MSB)
  while (Wire.available())
  {
    *buf-- = Wire.read();
  }
}

void printMac()
{
  unsigned char DEVEUI[EUI64_MAC_LENGTH];
  setDevEui(&DEVEUI[EUI64_MAC_LENGTH - 1]);

  Serial.print(F("DEVEUI: "));

  for (int count = EUI64_MAC_LENGTH; count > 0; count--)
  {
    Serial.print("0x");
    if (DEVEUI[count - 1] <= 0x0F) Serial.print("0");
    Serial.print(DEVEUI[count - 1], HEX);
    Serial.print(" ");
  }
  Serial.println();

}