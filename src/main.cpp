#include <Arduino.h>
#include <array>                // en.cppreference.com/w/cpp/container/array
#include <Wire.h>               // I2C comms support for IMU, Barometer
#include <SPI.h>                // SPI comms support for SD
#include <SD.h>                 // For SD Card
#include <Arduino_LSM9DS1.h>    // For LSM9DS01 IMU
#include <Adafruit_BMP280.h>    // For BMP280 Barometer
#include <constants.h>          // Constants
#include <helpers.h>            // Helper objects
#include <state.h>              // State class
#include <mean_sensor_filter.h> // MeanSensorFilter class

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
String logFile = "";    // Path on SD card to current log file.
MeanSensorFilter filter(25);

void setup() {
  // Begin serial connection.
  Serial.begin(SERIAL_TERM_BAUD);
  while (!Serial);

  // Wait for 5 seconds to allow for terminal connection.
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  // For debug purposes only... lists all available I2C devices.
  search_I2C_bus();

  if (!IMU.begin()) {
    logErr("Failed IMU initialization!");
    while (1);
  }

  if (!bmp.begin()) {
    logErr("Failed BMP280 initialization!");
    while (1);
  }

  // Set default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  {  // Init SD file IO
    logMsg("\nSD card init...");
    if (!SD.begin(SD_CS_PIN)) {
      logErr("SD Card failed, or not present");
      while (1) {
      }
    }
    logMsg("SD card initialized");
  }

  {  // Find out what the latest log file is, and use the next one
    int num = 0;
    while (logFile == "") {
      if (SD.exists("log" + (String)num + ".csv"))
        num++;
      else
        logFile = "log" + (String)num + ".csv";
    }
    logMsg("Using log file: " + (String)logFile);
  }

  {  // Send header line
    File log = SD.open(logFile, FILE_WRITE);
    if (log) {
      log.println(State::header_line());
      log.close();
      logMsg("CSV log header line written");
    } else
      logErr("Error opening " + (String)logFile);
  }
}

void loop() {
  // CodeTimer("void loop()");
  unsigned long begin = millis();
  std::array<double, 3> acc_raw;  // Acceleration (m/s^2)
  std::array<double, 3> gyro_raw; // Angular velocity (deg/s)
  std::array<double, 3> mag_raw;  // Magnetometer values (uT)
  double press_raw;               // Atmospheric pressure (hPa)
  double temp_raw;                // Ambient temperature (C)

  IMU.readAcceleration(acc_raw[0], acc_raw[1], acc_raw[2]);
  IMU.readGyroscope(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
  IMU.readMagneticField(mag_raw[0], mag_raw[1], mag_raw[2]);
  // Adjust acceleration measurements from g's to m/s^2.
  for(uint8_t i = 0; i < 3; i++) acc_raw[i] = acc_raw[i] * 9.81;

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  temp_raw = temp_event.temperature;
  press_raw = pressure_event.pressure;

  State state(acc_raw, gyro_raw, mag_raw, press_raw, temp_raw, acc_raw, gyro_raw, mag_raw, press_raw, temp_raw);
  filter.add_data(&state);
  filter.calculate_filter(&state);

  {  // Write state info to SD file, log to serial
    File log = SD.open(logFile, FILE_WRITE);
    if (log) {
      String msg = state.format_log_line();
      logMsg(msg);
      log.println(msg);
      log.close();
    } else
      logErr("Error opening " + (String)logFile);
  }

  delay(10);
  logMsg("Loop took: " + (String)(millis() - begin) + "ms");
}