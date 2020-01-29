#include <Arduino.h>
#include "IMU.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = PB0;

#define DT 0.02       // Loop time
#define AA 0.97       // complementary filter constant
#define G_GAIN 0.070  // [deg/s/LSB]

byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;  // [deg/s]
float rate_gyr_x = 0.0;  // [deg/s]
float rate_gyr_z = 0.0;  // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;

unsigned long startTime;

void setup() {
  // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  // while (!Serial.available()) delay(1);
  for (uint32_t i = 0; i < 10; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  detectIMU();
  enableIMU();

  // SD Card demo
  {
    Serial.print("\nInitializing SD card...");

    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, chipSelect)) {
      Serial.println("initialization failed. Things to check:");
      Serial.println("* is a card inserted?");
      Serial.println("* is your wiring correct?");
      Serial.println("* did you change the chipSelect pin to match your shield or module?");
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
  startTime = millis();

  // Read the measurements from  sensors
  readACC(buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));

  readMAG(buff);
  magRaw[0] = (int)(buff[0] | (buff[1] << 8));
  magRaw[1] = (int)(buff[2] | (buff[3] << 8));
  magRaw[2] = (int)(buff[4] | (buff[5] << 8));

  readGYR(buff);
  gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));
  gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));

  // Convert Gyro raw to degrees per second
  rate_gyr_x = (float)gyrRaw[0] * G_GAIN;
  rate_gyr_y = (float)gyrRaw[1] * G_GAIN;
  rate_gyr_z = (float)gyrRaw[2] * G_GAIN;

  // Calculate the angles from the gyro
  gyroXangle += rate_gyr_x * DT;
  gyroYangle += rate_gyr_y * DT;
  gyroZangle += rate_gyr_z * DT;

  // Convert Accelerometer values to degrees
  AccXangle = (float)(atan2(accRaw[1], accRaw[2]) + M_PI) * RAD_TO_DEG;
  AccYangle = (float)(atan2(accRaw[2], accRaw[0]) + M_PI) * RAD_TO_DEG;

  // If IMU is up the correct way, use these lines
  AccXangle -= (float)180.0;
  if (AccYangle > 90)
    AccYangle -= (float)270;
  else
    AccYangle += (float)90;

  // Complementary filter used to combine the accelerometer and gyro values.
  CFangleX = AA * (CFangleX + rate_gyr_x * DT) + (1 - AA) * AccXangle;
  CFangleY = AA * (CFangleY + rate_gyr_y * DT) + (1 - AA) * AccYangle;

  // Compute heading
  float heading = 180 * atan2(magRaw[1], magRaw[0]) / M_PI;

  // Convert heading to 0 - 360
  if (heading < 0) heading += 360;

  Serial.print("AccX ");
  Serial.print(AccXangle);
  Serial.print("\tAccY ");
  Serial.print(AccYangle);

  Serial.print("\tGyrX ");
  Serial.print(gyroXangle);
  Serial.print("\tGyrY ");
  Serial.print(gyroYangle);
  Serial.print("\tGyrZ ");
  Serial.print(gyroZangle);
  Serial.print("\tCFangleX ");
  Serial.print(CFangleX);
  Serial.print("\tCFangleY ");
  Serial.print(CFangleY);
  Serial.print("\theading ");
  Serial.print(heading);
  Serial.print("\tLoop Time ");

  // Each loop should be at least 20ms.
  while (millis() - startTime < (DT * 1000)) {
    delay(1);
  }
  Serial.println(millis() - startTime);
}
