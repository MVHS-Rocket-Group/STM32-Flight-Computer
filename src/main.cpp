#include <Adafruit_BMP280.h>  // For BMP280 Barometer
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>     // For LSM9DS01 IMU
#include <SD.h>                  // For SD Card
#include <SPI.h>                 // SPI comms support for SD
#include <Wire.h>                // I2C comms support for IMU, Barometer
#include <constants.h>           // Constants
#include <helpers.h>             // Helper objects
#include <mean_sensor_filter.h>  // MeanSensorFilter class
#include <state.h>               // State class
#include <array>                 // en.cppreference.com/w/cpp/container/array

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
String logFile = "";  // Path on SD card to current log file.
MeanSensorFilter filter(25);
State previous_timestep;

unsigned int esc_calibration_begin;

void setup() {
  // Begin serial connection.
  Serial.begin(SERIAL_TERM_BAUD);
  while (!Serial) delayMicroseconds(1);

  {  // I/O pins setup.
    analogWriteResolution(PWM_WRITE_RES);
    analogWriteFrequency(PWM_FREQ);
    pinMode(PWM_POD1_PIN, OUTPUT);
    pinMode(PWM_POD2_PIN, OUTPUT);
    pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
    pinMode(EXP_LED_R_PIN, OUTPUT);
    pinMode(EXP_LED_G_PIN, OUTPUT);
    pinMode(EXP_LED_B_PIN, OUTPUT);
    pinMode(EXT_LED_R_PIN, OUTPUT);
    pinMode(EXT_LED_G_PIN, OUTPUT);
    pinMode(EXT_LED_B_PIN, OUTPUT);
  }

  // Wait for 5 seconds to allow for terminal connection.
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  // Lists all available I2C devices for debug purposes.
  detect_I2C_devices();

  {  // I2C sensor init.
    logMsg("Begin: initializing IMU...");
    if (!IMU.begin()) {
      logErr("Failed IMU initialization!");
      // Set error code on external LED.
      set_ext_led_color(RGB_YELLOW);
      while (1) delayMicroseconds(1);
    } else
      logMsg("End: initializing IMU.");

    logMsg("Begin: initializing BMP280...");
    if (!bmp.begin()) {
      logErr("Failed BMP280 initialization!");
      // Set error code on external LED.
      set_ext_led_color(RGB_YELLOW);
      while (1) delayMicroseconds(1);
    } else
      logMsg("End: initializing BMP280.");
  }

  // Set default settings for BMP280. (from datasheet)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  {  // SPI SD card init.
    logMsg("Begin: SD card init...");
    if (!SD.begin(SD_CS_PIN)) {
      logErr("SD Card failed, or not present");
      // Set error code on external LED.
      set_ext_led_color(RGB_YELLOW);
      while (1) delayMicroseconds(1);
    }
    logMsg("End: SD card init.");
  }

  {  // Log file init.
    // Find out what the latest log file is, and use the next one
    int num = 0;
    while (logFile == "") {
      if (SD.exists("log" + (String)num + ".csv"))
        num++;
      else
        logFile = "log" + (String)num + ".csv";
    }
    logMsg("Using log file: " + (String)logFile);

    // Send header line
    File log = SD.open(logFile, FILE_WRITE);
    if (log) {
      log.println(State::header_line());
      log.close();
      logMsg("CSV log header line written");
    } else
      logErr("Error opening " + (String)logFile);
  }

  // Initialize first loop iteration with DISARMED State.
  previous_timestep._next_flight_state = DISARMED;
}

void loop() {
  // CodeTimer("void loop()");
  unsigned long begin = millis();
  std::array<double, 3> acc_raw;   // Acceleration (m/s^2)
  std::array<double, 3> gyro_raw;  // Angular velocity (deg/s)
  std::array<double, 3> mag_raw;   // Magnetometer values (uT)
  double press_raw;                // Atmospheric pressure (hPa)
  double temp_raw;                 // Ambient temperature (C)

  {  // Read data from sensors.
    IMU.readAcceleration(acc_raw[0], acc_raw[1], acc_raw[2]);
    IMU.readGyroscope(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    IMU.readMagneticField(mag_raw[0], mag_raw[1], mag_raw[2]);
    // Adjust acceleration measurements from g's to m/s^2.
    acc_raw *= 9.81;

    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    temp_raw = temp_event.temperature;
    press_raw = pressure_event.pressure;
  }

  // Save recorded data to state vector, update filter.
  State current_timestep(acc_raw, gyro_raw, mag_raw, press_raw, temp_raw,
                         acc_raw, gyro_raw, mag_raw, press_raw, temp_raw);
  filter.add_data(&current_timestep);
  filter.calculate_filter(&current_timestep);

  // Set default color for external LED in state vector.
  current_timestep._external_led = {0, 0, 0};

  // Flight control loop.
  switch (previous_timestep._next_flight_state) {
    case DISARMED:
      // Dummy case, only to be used if/when arming switch implemented.
      current_timestep._next_flight_state = CALIBRATING_ESC;
      break;

    case CALIBRATING_ESC:
      // ESC startup protocol: MIN_COMMAND for 3 seconds to let ESC
      // self-calibrate.
      logMsg("Begin: ESC MIN_COMMAND calibration...");
      esc_calibration_begin = millis();

      analogWrite(PWM_POD1_PIN, PWM_MIN_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MIN_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MIN_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MIN_DUTY;

      set_ext_led_color(RGB_BLUE);
      current_timestep._external_led = {RGB_BLUE};

      // After 3 seconds, the program contunues and jumps to ARMED state.
      if(millis() - esc_calibration_begin >= 3000) {
        current_timestep._next_flight_state = ARMED;
        logMsg("END: ESC MIN_COMMAND calibration.");
      }
      break;

    case ARMED:
      analogWrite(PWM_POD1_PIN, PWM_MIN_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MIN_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MIN_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MIN_DUTY;

      set_ext_led_color(RGB_RED);
      current_timestep._external_led = {RGB_RED};

      // TODO: Is the IMU y-axis vertical to the rocket?
      // Move to POWERED_ASSENT when IMU sees >2g's of vertical acceleration.
      if (current_timestep._acc_f[1] > 9.81 * 2) {
        current_timestep._next_flight_state = POWERED_ASSENT;
      }
      break;

    case POWERED_ASSENT:
      // TODO: Do we want full power on the motors?
      analogWrite(PWM_POD1_PIN, PWM_MAX_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MAX_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MAX_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MAX_DUTY;

      set_ext_led_color(RGB_GREEN);
      current_timestep._external_led = {RGB_GREEN};

      // TODO: Is the IMU y-axis vertical to the rocket?
      // Move to BALLISTIC_TRAJECTORY when IMU sees <2g's of vertical
      // acceleration.
      if (current_timestep._acc_f[1] < 9.81 * 2) {
        current_timestep._next_flight_state = BALLISTIC_TRAJECTORY;
      }
      break;

    case BALLISTIC_TRAJECTORY:
      // TODO: Do we want full power on the motors?
      analogWrite(PWM_POD1_PIN, PWM_MAX_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MAX_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MAX_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MAX_DUTY;

      set_ext_led_color(RGB_GREEN);
      current_timestep._external_led = {RGB_GREEN};

      // TODO: Is this a good trigger point?
      // Move to CHUTE_DEPLOYED when IMU sees a violent jerk from chute release.
      if (current_timestep.acc_magnitude(false) > 9.81 * 2) {
        current_timestep._next_flight_state = CHUTE_DEPLOYED;
      }
      break;

    case CHUTE_DEPLOYED:
      analogWrite(PWM_POD1_PIN, PWM_MIN_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MIN_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MIN_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MIN_DUTY;

      set_ext_led_color(RGB_RED);
      current_timestep._external_led = {RGB_RED};

      // TODO: Is this a good trigger point?
      // Move to LANDED when IMU sees a violent jerk from landing.
      if (current_timestep.acc_magnitude(false) > 9.81 * 2) {
        current_timestep._next_flight_state = LANDED;
      }
      break;

    case LANDED:
      // Dummy case, sit here and wait for rocket to be retrieved.
      analogWrite(PWM_POD1_PIN, PWM_MIN_DUTY);
      analogWrite(PWM_POD2_PIN, PWM_MIN_DUTY);
      current_timestep._esc_pod_1_pwm = PWM_MIN_DUTY;
      current_timestep._esc_pod_2_pwm = PWM_MIN_DUTY;

      set_ext_led_color(RGB_RED);
      current_timestep._external_led = {RGB_RED};
      break;

    default:
      logErr("Unknown FlightState type detected!");
      break;
  }

  // TODO: Is the IMU y-axis vertical to the rocket?
  // Experiment LED controller.
  if (within(current_timestep._acc_f[1], 9.81, 0.10)) {
    set_exp_led_color(RGB_RED);
    current_timestep._experiment_led = {RGB_RED};
  } else if (within(current_timestep._acc_f[1], 0, 0.10)) {
    set_exp_led_color(RGB_GREEN);
    current_timestep._experiment_led = {RGB_GREEN};
  } else if (current_timestep._acc_f[1] < 0.0) {
    set_exp_led_color(RGB_BLUE);
    current_timestep._experiment_led = {RGB_BLUE};
  } else {
    set_exp_led_color(RGB_YELLOW);
    current_timestep._experiment_led = {RGB_YELLOW};
  }

  {  // Write state info to SD file, log to serial
    File log = SD.open(logFile, FILE_WRITE);
    if (log) {
      String msg = current_timestep.format_log_line();
      logMsg(msg);
      log.println(msg);
      log.close();
    } else
      logErr("Error opening " + (String)logFile);
  }

  previous_timestep = current_timestep;

  delay(10);
  logMsg("Loop took: " + (String)(millis() - begin) + "ms");
}