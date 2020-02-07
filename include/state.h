#ifndef STATE_H
#define STATE_H
#include <Arduino.h>
#include <constants.h>        // Constants
#include <helpers.h>          // Helper objects
#include <array>

enum FlightState { ON_PAD, LAUNCHED, IN_FREEFALL, LANDED};
// https://stackoverflow.com/a/5094430/3339274
inline String FlightState_to_string(FlightState state) {
    switch (state) {
        case ON_PAD:      return "ON_PAD";
        case LAUNCHED:    return "LAUNCHED";
        case IN_FREEFALL: return "IN_FREEFALL";
        case LANDED:      return "LANDED";
        default:          return "[Unknown FlightState type]";
    }
}

struct State {
 public:
  double _time;       // Time since boot (millis)
  FlightState _flight_state;

  // Raw sensor readings
  std::array<double, 3> _acc_raw;          // Acceleration (m/s^2)
  std::array<double, 3> _gyro_raw;         // Angular velocity (deg/s)
  std::array<double, 3> _mag_raw;          // Magnetometer values (uT)

  double _press_raw;           // Atmospheric pressure (hPa)
  double _temp_raw;            // Ambient temperature (C)

  // Filtered sensor readings (rolling average)
  std::array<double, 3> _acc_f;            // Acceleration (m/s^2)
  std::array<double, 3> _gyro_f;           // Angular velocity (deg/s)
  std::array<double, 3> _mag_f;            // Magnetometer values (uT)

  double _press_f;             // Atmospheric pressure (hPa)
  double _temp_f;              // Ambient temperature (C)

  // Auto-populate fields from sensors.
  // TODO: Implement FlightState!
  State(std::array<double, 3> acc_raw, std::array<double, 3> gyro_raw,
        std::array<double, 3> mag_raw, double press_raw, double temp_raw,
        std::array<double, 3> acc_f, std::array<double, 3> gyro_f,
        std::array<double, 3> mag_f, double press_f, double temp_f) :
          _acc_raw(acc_raw), _gyro_raw(gyro_raw), _mag_raw(mag_raw),
          _press_raw(press_raw), _temp_raw(temp_raw), _acc_f(acc_f),
          _gyro_f(gyro_f), _mag_f(mag_f), _press_f(press_f), _temp_f(temp_f) {
    _time = millis() / 1000.0;
  }

  static String header_line() {
    // https://stackoverflow.com/a/3859167/3339274
    return "time, flight_state, acc_x_raw, acc_y_raw, acc_z_raw, gyro_x_raw, "
           "gyro_y_raw, gyro_z_raw, mag_x_raw, mag_y_raw, mag_z_raw, press_raw, "
           "temp_raw, acc_x_f, acc_y_f, acc_z_f, gyro_x_f, gyro_y_f, gyro_z_f, "
           "mag_x_f, mag_y_f, mag_z_f, press_f, temp_f";
  }

  String format_log_line() {
    return (String)_time + ", " + FlightState_to_string(_flight_state) + ", " +
           (String)_acc_raw[0] + ", " + (String)_acc_raw[1] + ", " + (String)_acc_raw[2] + ", " +
           (String)_gyro_raw[0] + ", " + (String)_gyro_raw[1] + ", " + (String)_gyro_raw[2] + ", " +
           (String)_mag_raw[0] + ", " + (String)_mag_raw[1] + ", " + (String)_mag_raw[2] + ", " +
           (String)_press_raw + ", " + (String)_temp_raw;
  }
};

#endif