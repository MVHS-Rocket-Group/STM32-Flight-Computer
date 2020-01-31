#ifndef HELPERS_H
#define HELPERS_H
#include <Arduino.h>
#include <Wire.h>
#include <constants.h>

// Searches for devices at all available addresses on the I2C bus and prints results.
void search_I2C_bus() {
  int count = 0;
  Wire.begin();
  Serial.println("Searching for I2C Devices...");
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) {
      Serial.print("\tFound: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
}

// Log the debug message only if the DEBUG flag is set
inline void logMsg(String msg) {
  if (DEBUG_MODE) Serial.println("DEBUG: " + msg);
}
inline void logErr(String msg) { Serial.println("Error: " + msg); }

#endif
