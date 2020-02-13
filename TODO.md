# TODO

~~Strikethrough~~ means an item is complete.

- Flight controller (*PWM Output*)
- ~~Flight state data logger~~
- Camera recorder via “pressing” the record button?
- Landing buzzer control?
- Detect arming switch toggle with pin-change interrupt
  - Move ESC calibration to `void loop()`, only execute once armed
  - Trigger transition from `DISARMED` to `CALIBRATING_ESC`
- ~~Implement `CALIBRATING` and `DISARMED` states in `FlightState` enum type~~
  - ~~Rename `ON_PAD` state to `ARMED`~~
- Implement `FlightState`-based control loop in `void loop()` with `switch()` block
  - Add goal items to the state vector
    - Duty cycle goal for fan pods is only != MIN_COMMAND when in `POWERED_ASSENT`and `BALLISTIC_TRAJECTORY` states
  - Save timestamp for each state transition into a global variable
  - `DISARMED` --> `CALIBRATING_ESC`: when arming switch triggered (LOW --> HIGH)
  - `CALIBRATING_ESC` --> `ARMED`: when 3-second ESC calibration is complete
  - `ARMED` --> `POWERED_ASSENT`: when IMU sees >2g's of vertical acceleration
  - `POWERED_ASSENT` --> `BALLISTIC_TRAJECTORY`: when IMU sees <2g's of vertical acceleration
  - `BALLISTIC TRAJECTORY` --> `CHUTE_DEPLOYED`: when IMU sees violent jerk of deployment mech
  - `CHUTE_DEPLOYED` --> `LANDED`: when IMU sees violent jerk of landing
- Implement event tracking, add as detected to state vector
  - Decide what events are worth noting down
- Verify that pin assignments in `constants.h` match reality
- ~~Configure TravisCI~~
- Implement RGB status LED control (*see: https://learn.adafruit.com/adafruit-arduino-lesson-3-rgb-leds/arduino-sketch*)
