#ifndef HELPERS_H
#define HELPERS_H
#include <Arduino.h>
#include <Wire.h>
#include <constants.h>  // Constants
#include <state.h>      // State class
#include <array>

// Scans for devices at all available addresses on the I2C bus and prints
// results. Common I2C address associations:
// https://cdn-learn.adafruit.com/downloads/pdf/i2c-addresses.pdf
void detect_I2C_devices() {
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
inline void logMsg(const String& msg) {
  if (DEBUG_MODE) Serial.println("DEBUG: " + msg);
}
inline void logErr(const String& msg) { Serial.println("Error: " + msg); }

class CodeTimer {
 public:
  CodeTimer(const String& name) : _name(name) {
    Serial.println("Begin execution of \"" + _name + "\".");
    _begin = micros();
  }

  ~CodeTimer() {
    unsigned long duration = micros() - _begin;
    Serial.println("\"" + _name + "\" execution took " + duration + "µs.");
  }

 private:
  unsigned long _begin;
  String _name;
};

// Set EXPeriment package status LED.
void set_exp_led_color(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(EXP_LED_R_PIN, map(r, 0, 255, 0, PWM_RESOLUTION));
  analogWrite(EXP_LED_G_PIN, map(g, 0, 255, 0, PWM_RESOLUTION));
  analogWrite(EXP_LED_B_PIN, map(b, 0, 255, 0, PWM_RESOLUTION));
}

// Set EXTernal status LED.
void set_ext_led_color(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(EXT_LED_R_PIN, map(r, 0, 255, 0, PWM_RESOLUTION));
  analogWrite(EXT_LED_G_PIN, map(g, 0, 255, 0, PWM_RESOLUTION));
  analogWrite(EXT_LED_B_PIN, map(b, 0, 255, 0, PWM_RESOLUTION));
}

// Returns whether or not value is within +/- range of assertion. Range should
// be positive.
bool within(double value, double assertion, double range) {
  return std::abs(value - assertion) < range;
}

// Define overloads to the /=, *=, and += operators for std::array to help with
// vector arithmentic.
// https://stackoverflow.com/a/3595874/3339274

template <typename T, std::size_t SIZE>
std::array<T, SIZE>& operator/=(std::array<T, SIZE>& dividend,
                                const T& divisor) {
  for (unsigned int i = 0; i < dividend.size(); i++) dividend[i] /= divisor;
  return dividend;
}

template <typename T, std::size_t SIZE>
std::array<T, SIZE>& operator*=(std::array<T, SIZE>& arr, const T& scalar) {
  for (unsigned int i = 0; i < arr.size(); i++) arr[i] *= scalar;
  return arr;
}
template <typename T, std::size_t SIZE>
std::array<T, SIZE>& operator*=(const T& scalar, std::array<T, SIZE>& arr) {
  for (unsigned int i = 0; i < arr.size(); i++) arr[i] *= scalar;
  return arr;
}

template <typename T, std::size_t SIZE>
std::array<T, SIZE>& operator+=(std::array<T, SIZE>& arr1,
                                std::array<T, SIZE>& arr2) {
  for (unsigned int i = 0; i < arr1.size(); i++) arr1[i] += arr2[i];
  return arr1;
}

// Define overload to the + operator between types and String objects.

// https://stackoverflow.com/a/34106613/3339274
template <typename T>
String& operator+(String& str, T& item) {
  return str + (String)item;
}
template <typename T>
String& operator+(T& item, String& str) {
  return (String)item + str;
}

#endif
