#ifndef CONSTANTS_H
#define CONSTANTS_H

#define SD_CS_PIN PB0
#define SERIAL_TERM_BAUD 115200
#define DEBUG_MODE true

// TODO: Fix these to be correct!
#define ARM_SWITCH_PIN PA0
#define PWM_POD1_PIN PA1
#define PWM_POD2_PIN PA2
#define LED_R_PIN PA3
#define LED_G_PIN PA4
#define LED_B_PIN PA5
#define PWM_WRITE_RES 65535  // Change from default 8-bits to 16-bits

// https://electronicshobbyists.com/arduino-pwm-tutorial/
#define PWM_FREQ 1 / 0.02  // frequency (Hz) = 1 / period (sec)
#define PWM_MAX_DUTY \
  PWM_WRITE_RES * 2 / 20  // PWM value when firewalling the throttle.
#define PWM_MIN_DUTY PWM_WRITE_RES * 1 / 20  // PWM value when at idle throttle.

// RGB colors for the status LED.
#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define RGB_BLUE 0, 0, 255
#define RGB_YELLOW 255, 255, 0

#endif