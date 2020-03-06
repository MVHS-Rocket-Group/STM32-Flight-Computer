#ifndef CONSTANTS_H
#define CONSTANTS_H

#define SD_CS_PIN PB0
#define SERIAL_TERM_BAUD 115200
#define DEBUG_MODE true

#define ARM_SWITCH_PIN PA0
#define PWM_POD1_PIN PB3
#define PWM_POD2_PIN PB4

// EXPeriment package status LED.
#define EXP_LED_R_PIN PB14
#define EXP_LED_G_PIN PB13
#define EXP_LED_B_PIN PB12

// EXTernal status LED.
#define EXT_LED_R_PIN PA1
#define EXT_LED_G_PIN PA2
#define EXT_LED_B_PIN PA3

#define PWM_WRITE_RES 12  // Change from default 8-bits to 12-bits

#define PWM_WRITE_RES 12  // Change from default 8-bits to 12-bits
#define PWM_FREQ 1 / 0.02  // frequency (Hz) = 1 / period (sec)
#define PWM_MAX_DUTY pow(2, PWM_WRITE_RES) * 2 / 20  // PWM value when firewalling the throttle.
#define PWM_MIN_DUTY pow(2, PWM_WRITE_RES) * 1 / 20  // PWM value when at idle throttle.define PWM_POD2_PIN PB4

// RGB colors for the status LED.
#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define RGB_BLUE 0, 0, 255
#define RGB_YELLOW 255, 255, 0

#endif