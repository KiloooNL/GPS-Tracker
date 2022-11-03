/***
Simple GPS Tracker
https://github.com/KiloooNL/GPS-Tracker

Using the following hardware:
- Arduino Nano BLE 33 - https://core-electronics.com.au/arduino-nano-33-ble.html
- U-blox NEO-6M GPS Module - https://core-electronics.com.au/u-blox-neo-6m-gps-module.html 
- SD Card reader - https://core-electronics.com.au/sd-card-module.html

Using the following libraries:
- TinyGPS+
- Time
- SPI
- SdFat for exFAT support

This script will get the location, date and time and print it to a csv file on the sd card.
It will update every 10 seconds. If the location has not changed, it will check every 10 seconds for a change, and if none is found, wait until mobile again.
**/

// Include TinyGPS+ library https://github.com/mikalhart/TinyGPSPlus
#include <TinyGPSPlus.h>
#include <Time.h>
#include <SPI.h>

// SDFat library https://github.com/greiman/SdFat
#include <SdFat.h>

// Set up SDFat
#define SD_FAT_TYPE 0
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
// Arduino Nano = Pin 10
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = 10;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

// Set up serial
static const uint32_t SerialMonitorBaud = 115200;
static const uint32_t GPSBaud = 9600;

// The TinyGPS+ object
TinyGPSPlus gps;

void setup() {
  Serial.begin(SerialMonitorBaud);
  Serial1.begin(GPSBaud);

  // Wait until serial port is open
  while(!Serial);
  Serial.println('Serial is now online');
  while(!Serial1);
  Serial.println('Serial1 is now online');

  if(Serial1.available()) {
    Serial.println("Ready.");
    Serial.println("---------------");
  }

  Serial.println("Booting...");
  for(int i = 0; i < 10; i++) {
    Serial.print(".");
    delay(100);
  }

  // Initialize SD Card
  initSD();
}

// Start the loop. Only work if GPS data is valid.
void loop() {
  while (Serial1.available()) {
    if(gps.encode(Serial1.read())) {
      if(gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
        if(gps.date.year() != '2000') {
          displayInfo();

          String dataStr = ""; // data string for saving
          dataStr += String(gps.date.month())+"/"+String(gps.date.day())+"/"+
                          String(gps.date.year())+",";
          dataStr += String(gps.time.hour())+":"+String(gps.time.minute())+":"+
                          String(gps.time.second())+"."+String(gps.time.centisecond())+",";
          dataStr += String(gps.location.lat(),5)+","; // latitude
          dataStr += String(gps.location.lng(),5); // longitude
          
          writeFile(dataStr); // save new data points
        }
      }
    }
  }
}

// Display info in Serial Monitor
void displayInfo() {
      // Create some temp values for latitude and longitude for later
      float tempLat = 0;
      float tempLng = 0;

      if(gps.location.isValid()) {
        Serial.print("Location: ");
        Serial.print(gps.location.lat(), 4);
        Serial.print(",");
        Serial.println(gps.location.lng(), 4);

        // Store our current latitude and logitude
        tempLat = gps.location.lat();
        tempLng = gps.location.lng();
      } else {
        Serial.println("Location: Not Available");
      }

      Serial.print("Date: ");
      if (gps.date.isValid() && gps.date.year() != '2000') {
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
      } else {
        Serial.println("Not Available");
      }

      Serial.print("Time: ");
      if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(":");
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(":");
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(".");
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.println(gps.time.centisecond());
      } else {
        Serial.println("Not Available");
      }
      Serial.println();

      // Update every 10 seconds
      delay(10000);

      // Check if we are stationary by comparing the previous GPS location to our current location. If so, let's sleep a while...
      int i = 0;
      while((tempLat - gps.location.lat()) < 0.0001 || (tempLng -gps.location.lng()) < 0.00001) {
        if(i < 10) {
          if(i == 9) {
            Serial.println("Waited one minute. We must be stopped. Will resume logging when mobile.");
          } else {
            Serial.println("Currently stationary as location has not changed. Waiting 10 seconds and retrying.");
            delay(10000);
          } i++;
        } else {
          while(tempLat == (gps.location.lat(), 4) || tempLng == (gps.location.lng(), 4)) {
            // do nothing - wait until mobile!
          }
        }
      }
}

void initSD() {
  Serial.println("\nInitializing SD card...");
 
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    return;
  }

  // Create a header in the csv file
  Serial.println("Writing header to file.");
  writeFile("Date [mm/dd/yyyy], Time [HH:MM:SS.ZZ], Latitude [deg], Longitude [deg]");
}

// Function for writing data to SD card
void writeFile(String WriteData) {
  Serial.print("Writing the following data to file: ");
  Serial.println(WriteData);

  // Make sure our current time is valid, otherwise we are writing useless data
  if (gps.time.isValid() && gps.date.year() != '2000') {
    // Write file to SD card - format: YYMMDD
    if (file.open("TripLog.csv", FILE_WRITE)) {
      file.println(WriteData); // write data to file
      file.close(); // close file before continuing
    } else {
      delay(50); // prevents cluttering
      Serial.println("Unable to write file to SD card."); // print error if SD card issue
    }
  } else {
    Serial.println("Waiting for GPS date sync...");
    delay(5000);
  }
}

// Print a GPS dump for debugging
void GPSDump() {
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat(), 7); // Latitude in degrees (double)
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng(), 7); // Longitude in degrees (double)
  Serial.print("Date: ");
  Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
  Serial.print("Speed (mps): ");
  Serial.println(gps.speed.mps()); // Speed in meters per second (double)
  Serial.print("Speed (km/hr): ");
  Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
}
