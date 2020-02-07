#ifndef MEAN_SENSOR_FILTER_H
#define MEAN_SENSOR_FILTER_H
#include <Arduino.h>
#include <constants.h>
#include <helpers.h>
#include <array>
#include <vector>

class MeanSensorFilter {
 public:
  MeanSensorFilter(int max_history_size) : _max_history_size(max_history_size) {
    // Initialize the vectors off with the max history size so no reallocation
    // should be necessary.
    // https://stackoverflow.com/a/11457629/3339274
    _acc_history.reserve(_max_history_size);
    _gyro_history.reserve(_max_history_size);
    _mag_history.reserve(_max_history_size);
    _press_history.reserve(_max_history_size);
    _temp_history.reserve(_max_history_size);
  }

  void add_data(State* state) {
    // Maintain history size, remove first item if the buffer gets too long.
    if(_acc_history.size() >= _max_history_size) {
      // https://stackoverflow.com/a/875109/3339274
      _acc_history.erase(_acc_history.begin());
      _gyro_history.erase(_gyro_history.begin());
      _mag_history.erase(_mag_history.begin());
      _press_history.erase(_press_history.begin());
      _temp_history.erase(_temp_history.begin());
    }

    // Add new data to history.
    _acc_history.push_back(state->_acc_raw);
    _gyro_history.push_back(state->_gyro_raw);
    _mag_history.push_back(state->_mag_raw);
    _press_history.push_back(state->_press_raw);
    _temp_history.push_back(state->_temp_raw);
  }

  void calculate_filter(State* state) {
    // Create temporary objects to store running tallies and eventually averages.
    std::array<double, 3> acc_temp = {0.0, 0.0, 0.0};
    std::array<double, 3> gyro_temp = {0.0, 0.0, 0.0};
    std::array<double, 3> mag_temp = {0.0, 0.0, 0.0};
    double press_temp = 0.0;
    double temp_temp = 0.0;
    int current_buffer_size = _acc_history.size();

    // Fill temporary objects with a sum of all data points in the buffer.
    for(uint16_t i = 0; i < current_buffer_size; i++) {
      for(uint8_t j = 0; j < 3; j++) acc_temp[j] += _acc_history[i][j];
      for(uint8_t j = 0; j < 3; j++) gyro_temp[j] += _gyro_history[i][j];
      for(uint8_t j = 0; j < 3; j++) mag_temp[j] += _mag_history[i][j];

      press_temp += _press_history[i];
      temp_temp += _temp_history[i];
    }

    // Convert totals to averages...
    for(uint8_t j = 0; j < 3; j++) acc_temp[j] = acc_temp[j] / current_buffer_size;
    for(uint8_t j = 0; j < 3; j++) gyro_temp[j] = gyro_temp[j] / current_buffer_size;
    for(uint8_t j = 0; j < 3; j++) mag_temp[j] = mag_temp[j] / current_buffer_size;
    press_temp = press_temp / current_buffer_size;
    temp_temp = temp_temp / current_buffer_size;

    // Store averages into the state object.
    state->_acc_f = acc_temp;
    state->_gyro_f = gyro_temp;
    state->_mag_f = mag_temp;
    state->_press_f = press_temp;
    state->_temp_f = temp_temp;
  }

 private:
  unsigned int _max_history_size;

  std::vector<std::array<double, 3>> _acc_history;
  std::vector<std::array<double, 3>> _gyro_history;
  std::vector<std::array<double, 3>> _mag_history;

  std::vector<double> _press_history;
  std::vector<double> _temp_history;
};

#endif