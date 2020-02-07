#ifndef CONSTANTS_H
#define CONSTANTS_H

#define SD_CS_PIN PB0
#define SERIAL_TERM_BAUD 115200
#define DEBUG_MODE true

// TODO: Fix these to be correct!
#define ARM_SWITCH_PIN PD0
#define PWM_POD1_PIN PD1
#define PWM_POD2_PIN PD2
#define PWM_WRITE_RES 65535             // Change from default 8-bits to 16-bits

// https://electronicshobbyists.com/arduino-pwm-tutorial/
#define ESC_PWM_FREQ 1 / 0.02           // frequency (Hz) = 1 / period (sec)
#define ESC_MAX_DUTY = 100 * 2.0 / 20.0 // Duty cycle percentage when firewalling the throttle.
#define ESC_MIN_DUTY = 100 * 1.0 / 20.0 // Duty cycle percentage when at idle throttle.

// ESC startup protocol: MIN_COMMAND for 3 seconds to let ESC sample MIN_COMMAND

#endif