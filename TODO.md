# TODO

~~Strikethrough~~ means an item is complete.

- Flight controller (*PWM Output*)
- ~~Flight state data logger~~
  - [IMU board: Ozzymaker BerryIMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor)
  - [LSM9DS1 PIO Library](https://platformio.org/lib/show/6589/Arduino_LSM9DS1)
  - [BMP280 PIO Library](http://platformio.org/lib/show/528/Adafruit%20BMP280%20Library)
  - [SD PIO Library](http://platformio.org/lib/show/868/SD)
- Important flight events: e.g. Launch detection, arming of different systems, control loop decisions, deployments (detected via accelerometer edges?)
- Camera recorder via “pressing” the record button?
- Landing buzzer control?
- Arming switch toggle (*use software interrupt*)
- ~~Implement `CALIBRATING` and `DISARMED` states in `FlightState` enum type~~
  - ~~Rename `ON_PAD` state to `ARMED`~~
- Move ESC calibration to `void loop()`, only execute once armed
- Implement usage of `FlightState` in `void loop()`
- Implement event tracking, add as detected to state vector
- Verify that pin assignments in `constants.h` match reality
- ~~Configure TravisCI~~
- Implement RGB status LED control
