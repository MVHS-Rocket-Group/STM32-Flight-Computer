#ifndef HELPERS_H
#define HELPERS_H
#include <Arduino.h>
#include <Wire.h>
#include <constants.h>        // Constants
#include <state.h>            // State class
#include <array>

// Searches for devices at all available addresses on the I2C bus and prints results.
void search_I2C_bus() {
  int count = 0;
  Wire.begin();
  Serial.println("Searching for I2C Devices...");

  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("\tFound: ");
      Serial.print(i, DEC);
      Serial.print(" aka. 0x");
      Serial.println(i, HEX);
      count++;
      delay(1);
    }
  }

  Serial.println("Found " + (String)count + " device(s).");
}

// Log the debug message only if the DEBUG flag is set
inline void logMsg(String msg) {
  if (DEBUG_MODE) Serial.println("DEBUG: " + msg);
}
inline void logErr(String msg) { Serial.println("Error: " + msg); }

class CodeTimer {
 public:
  CodeTimer(String name) {
    _name = name;
    Serial.println("Begin execution of \"" + _name + "\" code.");
    _begin = micros();
  }

  ~CodeTimer() {
    Serial.print("\"" + _name + "\" code execution took ");
    Serial.print(micros() - _begin);
    Serial.println("µs.");
  }
 private:
  unsigned long _begin;
  String _name;
};

#endif
