#ifndef STATE_H
#define STATE_H
#include <Arduino.h>
#include <constants.h>  // Constants
#include <helpers.h>    // Helper objects
#include <array>        // en.cppreference.com/w/cpp/container/array
#include <cmath>        // Math operations

enum FlightState {
  DISARMED,
  CALIBRATING_ESC,
  ARMED,
  POWERED_ASSENT,
  BALLISTIC_TRAJECTORY,
  CHUTE_DEPLOYED,
  LANDED
};
// https://stackoverflow.com/a/5094430/3339274
inline String FlightState_text(FlightState state) {
  switch (state) {
    case DISARMED:
      return "DISARMED";
    case CALIBRATING_ESC:
      return "CALIBRATING_ESC";
    case ARMED:
      return "ARMED";
    case POWERED_ASSENT:
      return "POWERED_ASSENT";
    case BALLISTIC_TRAJECTORY:
      return "BALLISTIC_TRAJECTORY";
    case CHUTE_DEPLOYED:
      return "CHUTE_DEPLOYED";
    case LANDED:
      return "LANDED";
    default:
      return "[Unknown FlightState type]";
  }
}

struct State {
 public:
  double _time;                       // Time since boot (seconds)
  FlightState _current_flight_state;  // Flight state in current timestep
  FlightState _next_flight_state;     // Calculated flight state
  String _events_list;                // String list of event labels.

  // Raw sensor readings
  std::array<double, 3> _acc_raw;   // Acceleration (m/s^2)
  std::array<double, 3> _gyro_raw;  // Angular velocity (deg/s)
  std::array<double, 3> _mag_raw;   // Magnetometer values (uT)
  double _press_raw;                // Atmospheric pressure (hPa)
  double _temp_raw;                 // Ambient temperature (C)

  // Filtered sensor readings (rolling average)
  std::array<double, 3> _acc_f;   // Acceleration (m/s^2)
  std::array<double, 3> _gyro_f;  // Angular velocity (deg/s)
  std::array<double, 3> _mag_f;   // Magnetometer values (uT)
  double _press_f;                // Atmospheric pressure (hPa)
  double _temp_f;                 // Ambient temperature (C)

  // Goals/outputs
  std::array<int, 3> _external_led;    // R, G, B state sent to LED
  std::array<int, 3> _experiment_led;  // R, G, B state sent to LED
  int _esc_pod_1_pwm;                  // PWM value sent to ESC pod 1
  int _esc_pod_2_pwm;                  // PWM value sent to ESC pod 2

  // Quick and dirty contstructor, doesn't set anything but time.
  State() { _time = millis() / 1000.0; }

  // Takes in sensor readings, leaving goals/outputs to be manually set later.
  State(std::array<double, 3>& acc_raw, std::array<double, 3>& gyro_raw,
        std::array<double, 3>& mag_raw, double press_raw, double temp_raw,
        std::array<double, 3>& acc_f, std::array<double, 3>& gyro_f,
        std::array<double, 3>& mag_f, double press_f, double temp_f)
      : _acc_raw(acc_raw),
        _gyro_raw(gyro_raw),
        _mag_raw(mag_raw),
        _press_raw(press_raw),
        _temp_raw(temp_raw),
        _acc_f(acc_f),
        _gyro_f(gyro_f),
        _mag_f(mag_f),
        _press_f(press_f),
        _temp_f(temp_f) {
    _time = millis() / 1000.0;
  }

  // https://stackoverflow.com/a/117458/3339274
  void add_event(const String& event) { _events_list += event; }

  static String header_line() {
    // https://stackoverflow.com/a/3859167/3339274
    return "time, flight_state, acc_x_raw, acc_y_raw, acc_z_raw, gyro_x_raw, "
           "gyro_y_raw, gyro_z_raw, mag_x_raw, mag_y_raw, mag_z_raw, "
           "press_raw, "
           "temp_raw, acc_x_f, acc_y_f, acc_z_f, gyro_x_f, gyro_y_f, gyro_z_f, "
           "mag_x_f, mag_y_f, mag_z_f, press_f, temp_f, external_led, "
           "experiment_led, esc_pod_1_pwm, esc_pod_2_pwm, events";
  }

  double acc_magnitude(bool filtered) const {
    if (filtered)
      return sqrt(pow(_acc_f[0], 2) + pow(_acc_f[1], 2) + pow(_acc_f[2], 2));
    else
      return sqrt(pow(_acc_raw[0], 2) + pow(_acc_raw[1], 2) +
                  pow(_acc_raw[2], 2));
  }

  double gyro_magnitude(bool filtered) const {
    if (filtered)
      return sqrt(pow(_gyro_f[0], 2) + pow(_gyro_f[1], 2) + pow(_gyro_f[2], 2));
    else
      return sqrt(pow(_gyro_raw[0], 2) + pow(_gyro_raw[1], 2) +
                  pow(_gyro_raw[2], 2));
  }

  double mag_magnitude(bool filtered) const {
    if (filtered)
      return sqrt(pow(_mag_f[0], 2) + pow(_mag_f[1], 2) + pow(_mag_f[2], 2));
    else
      return sqrt(pow(_mag_raw[0], 2) + pow(_mag_raw[1], 2) +
                  pow(_mag_raw[2], 2));
  }

  String format_log_line() const {
    // First cast to String type is required to start off the expression with
    // a String type to concatonate onto. This is because it is illegal in C++
    // to define an addition operator involving two non-"user-defined" types,
    // e.g. double and const char[3] like in the case below.
    return (String)_time + ", " + FlightState_text(_current_flight_state) +
           ", " +

           _acc_raw[0] + ", " + _acc_raw[1] + ", " + _acc_raw[2] + ", " +
           _gyro_raw[0] + ", " + _gyro_raw[1] + ", " + _gyro_raw[2] + ", " +
           _mag_raw[0] + ", " + _mag_raw[1] + ", " + _mag_raw[2] + ", " +
           _press_raw + ", " + _temp_raw + ", " +

           _acc_f[0] + ", " + _acc_f[1] + ", " + _acc_f[2] + ", " + _gyro_f[0] +
           ", " + _gyro_f[1] + ", " + _gyro_f[2] + ", " + _mag_f[0] + ", " +
           _mag_f[1] + ", " + _mag_f[2] + ", " + _press_f + ", " + _temp_f +
           ", " +

           format_arr(_external_led) + ", " + format_arr(_experiment_led) +
           ", " + _esc_pod_1_pwm + ", " + _esc_pod_2_pwm + ", " +

           (_events_list == "" ? "[None]" : _events_list);
  }

 private:
  // Simple space-separated std::array string formatter.
  template <typename T, std::size_t SIZE>
  static String format_arr(const std::array<T, SIZE>& arr) {
    String output = "\"";

    for (unsigned int i = 0; i < arr.size() - 1; i++)
      output += (String)arr[i] + " ";

    output += (String)arr[arr.size() - 1] + "\"";

    return output;
  }
};

#endif