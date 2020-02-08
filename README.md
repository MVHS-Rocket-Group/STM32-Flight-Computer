# STM32-Flight-Computer

![Travis-CI Build Status](https://travis-ci.com/MVHS-Rocket-Group/STM32-Flight-Computer.svg?branch=master))

## Repository Structure

- `.pio/*`: PlatformIO config files and build directory. (*Generated upon execution of  first build*)
- `.vscode/*`: VSCode config files.
- `include/*`: C/C++ header files.
  - `constants.h`: Provides constant definitions.
  - `helpers.h`: Provides development infrastructure for project.
  - `mean_sensor_filter.h`: Lightweight sensor noise filter taking a time-average for all inputs.
  - `state.h`: State `struct` providing convenient storage for all values in the state vector.
- `lib/*`: C/C++ libraries.
  - `Arduino_LSM9DS1_ID6589/*`: Slightly modified version of [this library from PlatformIO registry](https://platformio.org/lib/show/6589/Arduino_LSM9DS1) tweaked to use the correct addresses for the `LSM9DS1` chip as configured by Ozzymaker.
- `src/*`: C/C++ source code files.
  - `main.cpp`: Primary entry point for program.
- `.gitattributes`, `.gitignore`: Git SCM config files.
- `travis.yml`: Travis CI (*continuous integration*) automated build checker config.
- `platformio.ini`: PlatformIO build config.
- `STM32-Flight-Computer.code-workspace`: VSCode workspace file.
- `TODO.md`: TODO list for project.

## Description

STM32 (*Blue Pill development board, `STM32F103C8T6` MCU part*) flight computer and FDR for high-power SRM rockets. This program is developed in conjunction with the [PlatformIO](https://platformio.org) IDE system with integration into VSCode and Atom editors, a substantial upgrade from the oxymoron Arduino IDE.

Program upload and line-by-line debugging support is provided by the wonderful [ST-Link V2](https://smile.amazon.com/Aideepen-ST-Link-Programming-Emulator-Downloader/dp/B01J7N3RE6) connected to the STM32's SWD (***S**erial **W**ire **D**ebug*) port and a serial text terminal by an [FTDI breakout board](https://smile.amazon.com/HiLetgo-FT232RL-Converter-Adapter-Breakout/dp/B00IJXZQ7C) connected to `UART1` on the STM32.

### Relevant Peripherals

- [Ozzymaker BerryIMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor) via I<sup>2</sup>C
- [SD Card breakout board](https://smile.amazon.com/SenMod-Adapter-Reader-Module-Arduino/dp/B01JYNEX56) via SPI
- [3v3 to 5v signal converter](https://smile.amazon.com/Logic-Converter-Bi-Directional-Module-Arduino/dp/B014MC1OAG)

## Principal control flow

- [Arduino framework](https://docs.platformio.org/en/latest/frameworks/arduino.html) for [STM32 targets](https://docs.platformio.org/en/latest/platforms/ststm32.html) in [PlatformIO](https://platformio.org)
- Control-loop paradigm with State and Goal messages logged.
- Gist of Features:
  - Flight controller (*PWM Output*)
  - Flight state data logger
    - [IMU board: Ozzymaker BerryIMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor)
    - [LSM9DS1 PIO Library](https://platformio.org/lib/show/6589/Arduino_LSM9DS1)
    - [BMP280 PIO Library](http://platformio.org/lib/show/528/Adafruit%20BMP280%20Library)
    - [SD PIO Library](http://platformio.org/lib/show/868/SD)
  - Important flight events: e.g. Launch detection, arming of different systems, control loop decisions, deployments (detected via accelerometer edges?)
  - Camera recorder via “pressing” the record button?
  - Landing buzzer control?
  - Arming switch toggle (*software interrupt*)

## Background on PWM control for Servo Motors and ESCs

RC PWM has a "window" period of 20ms (milliseconds), with a pulse ranging in width from 1ms to 2ms, where 1ms is ~0% command and 2ms is ~100% command. Duty cycle, a percentage, is a ratio of on-time to off-time.

![ESC PWM Diagram](https://upload.wikimedia.org/wikipedia/commons/b/b7/Sinais_controle_servomotor.JPG)

Therefore:

- 0% throttle command --> 5% duty cycle
- 100% throttle command --> 10% duty cycle

[Documentation on PWM usage in Arduino](https://electronicshobbyists.com/arduino-pwm-tutorial)

## Rocket IMU Axes

From perspective of a cockpit at the nose cone: (***TODO: Verify if these are still correct!***)

![Originally defined like a fighter plane due to how early spacecraft were flight cockpits plopped on the top of rocket boosters.](https://qph.fs.quoracdn.net/main-qimg-67b906f1ec6e62819e16134e76b8830f-c)

| Vehicle Axis: | Axis Description: | IMU Measurement Axis: |
|--------------:|-------------------|:----------------------|
| X | *roll - vertical axis through center of rocket* | +X (*acc*), +X (*gyro*) |
| Y | *pitch - horizontal axis* | -Y (*acc*), +Y (*gyro*) |
| Z | *yaw - horizontal axis* | -Z (*acc*), +Z (*gyro*) |

## Helpful Resources

- ESC
  - [ESC Specs](https://hobbyking.com/en_us/turnigy-monster-2000-200a-4-12s-brushless-esc.html)
    - [ESC manual](https://cdn-global-hk.hobbyking.com/media/file/969150300X462171X21.pdf)
    - [ESC programming card](https://hobbyking.com/en_us/turnigy-monster-2000-esc-programming-card.html)

## PlatformIO STM32 linker scripts

For some reason, the default `STSTM32` linker scripts for the generic `STM32F103C8`, the chip on the Blue Pill, assume that the the MCU has 64K of program flash memory, which is just not the case, as most boards around have 128K of flash. This shouldn't be an issue, but for the extra headroom, this is a good mod. [Original post on PlatformIO Community forum.](https://community.platformio.org/t/stm32f1-blue-pill-stuck-in-dfu-mode-after-upload/6853/19?u=ifconfig)

| File | Original | Post-mod |
|------|----------|----------|
| `[USER_DIR]\.platformio\platforms\ststm32\boards\genericSTM32F103C8.json` | `"maximum_size": 65536,` | `"maximum_size": 131072,` |
| `[USER_DIR]\.platformio\platforms\ststm32\ldscripts\stm32f103x8.ld` | `FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K` | `FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K` |
