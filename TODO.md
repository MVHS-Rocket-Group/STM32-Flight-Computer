# TODO

~~Strikethrough~~ means an item is complete.

## Incomplete

- Clear `// TODO:` annotations in `main.cpp` after discussion
- Implement `FlightState`-based control loop in `void loop()` with `switch()` block
  - ~~Add goal items to the state vector~~
    - ~~Duty cycle goal for fan pods is only != MIN_COMMAND when in `POWERED_ASSENT`and `BALLISTIC_TRAJECTORY` states~~
  - Place timeout on spinning motors after `POWERED_ASSENT` activated
  - Save timestamp for each state transition into a global variable
  - ~~Transition to `CALIBRATING_ESC`: when arming switch triggered (LOW --> HIGH)~~
  - ~~Transition to `ARMED`: when 3-second ESC calibration is complete~~
  - ~~Transition to `POWERED_ASSENT`: when IMU sees >2g's of vertical acceleration~~
  - ~~Transition to `BALLISTIC_TRAJECTORY`: when IMU sees <2g's of vertical acceleration~~
  - ~~Transition to `CHUTE_DEPLOYED`: when IMU sees violent jerk of deployment mech~~
  - ~~Transition to `LANDED`: when IMU sees violent jerk of landing~~
- Implement event tracking, add as detected to state vector
  - Decide what events are worth noting down

### Aspirations

- Camera recorder via “pressing” the record button?
- Landing buzzer control?

## Completed

- ~~Flight controller (*PWM Output*)~~
- ~~Flight state data logger~~
- ~~Implement `CALIBRATING` and `DISARMED` states in `FlightState` enum type~~
  - ~~Rename `ON_PAD` state to `ARMED`~~
- ~~Verify that pin assignments in `constants.h` match reality~~
- ~~Configure TravisCI~~
- ~~Implement RGB status LED control~~
  - ~~Add debug color for error code to EXTernal LED (*activated in `setup()` if I/O not initialized properly*)~~
  - ~~EXPeriment codes~~
    - ~~RED when >1g~~
    - ~~GREEN when within +/- .1g~~
    - ~~BLUE when negative <0g~~
  - ~~EXTernal codes~~
    - ~~RED when EXC calibration complete~~
    - ~~GREEN when fans turned on~~
    - ~~RED when fans off~~
  - ~~Create constants for pin assignments and colors in `constants.h`~~
  - ~~Implement helper function to write color to LED already~~
