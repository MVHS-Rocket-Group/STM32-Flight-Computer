#include <Arduino.h>
#include <Wire.h>             // I2C comms support for IMU, Barometer
#include <SPI.h>              // SPI comms support for SD
#include <SD.h>               // For SD Card
#include <Arduino_LSM9DS1.h>  // For LSM9DS01 IMU
#include <Adafruit_BMP280.h>  // For BMP280 Barometer
#include <constants.h>        // Constants
#include <helpers.h>          // Helper objects

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  // Begin serial connection.
  Serial.begin(SERIAL_TERM_BAUD);
  while (!Serial);

  // Wait for 5 seconds to allow for terminal connection.
  for (uint32_t i = 0; i < 5; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  search_I2C_bus();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  // SD Card demo.
  {
    Sd2Card card;
    SdVolume volume;
    SdFile root;

    Serial.print("\nInitializing SD card...");

    if (!card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
      Serial.println("initialization failed. Things to check:");
      while (1);
    } else {
      Serial.println("Wiring is correct and a card is present.");
    }

    // print the type of card
    Serial.println();
    Serial.print("Card type:         ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
      default:
        Serial.println("Unknown");
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
      Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
      while (1);
    }

    Serial.print("Clusters:          ");
    Serial.println(volume.clusterCount());
    Serial.print("Blocks x Cluster:  ");
    Serial.println(volume.blocksPerCluster());

    Serial.print("Total Blocks:      ");
    Serial.println(volume.blocksPerCluster() * volume.clusterCount());
    Serial.println();

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("Volume type is:    FAT");
    Serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    Serial.print("Volume size (Kb):  ");
    Serial.println(volumesize);
    Serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Gb):  ");
    Serial.println((float)volumesize / 1024.0);

    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);

    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
  }
}

void loop() {
  float x, y, z;

  IMU.readAcceleration(x, y, z);

  Serial.print("acc: (g's) ");
  Serial.print(x);
  Serial.print(' ');
  Serial.print(y);
  Serial.print(' ');
  Serial.print(z);

  IMU.readGyroscope(x, y, z);

  Serial.print("\tgyro: (deg/s) ");
  Serial.print(x);
  Serial.print(' ');
  Serial.print(y);
  Serial.print(' ');
  Serial.print(z);

  IMU.readMagneticField(x, y, z);

  Serial.print("\tmag: (uT) ");
  Serial.print(x);
  Serial.print(' ');
  Serial.print(y);
  Serial.print(' ');
  Serial.println(z);

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  Serial.print(F("\ttemp: (C) "));
  Serial.println(temp_event.temperature);

  Serial.print(F("\tPress: (hPa) "));
  Serial.println(pressure_event.pressure);

  delay(10);
}