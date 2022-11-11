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

// Onboard LED
#define RED 22     
#define BLUE 24     
#define GREEN 23
#define LED_PWR 25

void setup() {
  Serial.begin(SerialMonitorBaud);
  Serial1.begin(GPSBaud);
  
  // Initialize inbuilt LED as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the digital Pin as an output
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);

  // Wait until serial port is open
  while(!Serial1)
  Serial.println('Serial1 is now online');

  if(Serial1.available()) {
    Serial.println("Ready.");
    Serial.println("---------------");
  } else {
    Serial.println("Serial1 is not available.");
  }

  Serial.println("Booting...");
  for(int i = 0; i < 10; i++) {
    Serial.print(".");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
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
        // Even though we might be getting a 'valid' GPS time sync, if the year we receive is 2000, then the time sync has not completed.
        // In addition to this, wait until we are mobile (moving at over 5kmph)
        // 5kmph for a reason: sometimes while obtaining a fix the speed is innacurately reported (0-4.5kmph) while stationary
        if(gps.date.year() != '2000' && gps.speed.kmph() >= 5) {
          String dataStr = ""; // data string for saving
          dataStr += String(gps.date.month()) + "/"+ 
                     String(gps.date.day()) + "/" + 
                     String(gps.date.year()) + ",";
          dataStr += String(gps.time.hour()) + ":" + 
                     String(gps.time.minute()) + ":" + 
                     String(gps.time.second()) + "." + 
                     String(gps.time.centisecond()) + ",";
          dataStr += String(gps.location.lat(), 7) + ","; // latitude
          dataStr += String(gps.location.lng(), 7) + ","; // longitude
          dataStr += String(gps.speed.kmph()); // speed
          
          // Run displayInfo() first before we log any data. This will ensure we are moving before logging any data.
          displayInfo(); 

          digitalWrite(GREEN, HIGH);
          delay(500);  
          writeFile(dataStr);
          digitalWrite(GREEN, LOW);
          delay(500);

          // Run loop every 2.5 seconds
          delay(2500);
        }
      } else {
        if(!gps.location.isValid()) {
          Serial.println("Waiting on location sync.");
            digitalWrite(RED, HIGH);
            delay(500);  
            digitalWrite(RED, LOW);
            delay(500);
        }
        if(!gps.date.isValid()) {
          Serial.println("Waiting on date sync.");
        }
        if(!gps.time.isValid()) {
          Serial.println("Waiting on time sync.");
        }

        // Enable GPS dump when debugging
        // GPSDump();
        delay(2500);
      }
    }
  }
}

// Display info in Serial Monitor
void displayInfo() {
      if(gps.location.isValid()) {
        Serial.print("Location: ");
        Serial.print(gps.location.lat(), 7);
        Serial.print(",");
        Serial.println(gps.location.lng(), 7);
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
}

void initSD() {
  Serial.println("\nInitializing SD card...");
 
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Error initializing SD card...");
    sd.initErrorHalt(&Serial);
    return;
  } else {
    Serial.println("SD Card initialized.");
  }
}

// Function for writing data to SD card
void writeFile(String WriteData) {

  /** 
  Create a file, with a filename of today's date
  Data is stored in GMT time, so a post-conversion will have to be done.

  CSV Header is as follows:
  Date [mm/dd/yyyy], Time [HH:MM:SS.ZZ], Latitude [deg], Longitude [deg]
  **/

  Serial.print("Writing the following data to file: ");
  Serial.println(WriteData);

  // Make sure our current time is valid, otherwise we are writing useless data
  if (gps.time.isValid() && gps.date.year() != '2000') {
    // Write file to SD card - format: YYMMDD
    String name = "";
    name += String(gps.date.year()) + String(gps.date.month()) + String(gps.date.day()) + ".csv";
    
    if (file.open(name.c_str(), FILE_WRITE)) {
      file.println(WriteData); // write data to file
      file.close(); // close file before continuing 
    } else {
      delay(50); // prevents cluttering
      Serial.println("Unable to write file to SD card."); // print error if SD card issue
    }
  } else {
    Serial.println("Waiting for GPS date sync...");
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
