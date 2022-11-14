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

Error codes are shown by the red and blue LED and are as follows
1 long red flash = 10 unit
1 short blue flash = 1 unit
For example: 1 long, 2 short = error code 12

11: Serial error (No communication from GPS unit)
12: SD Card error (No card, wrong chip select pin, or wiring error)
13: Waiting for location sync
14: Waiting on date sync
15: Waiting on time sync
21: SD card write error
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
  pinMode(BLUE, OUTPUT);
  pinMode(LED_PWR, OUTPUT);

  // The power LED and the built-in LED are active-high. 
  // The three LEDs in the RGB LED, however, are active-low, hence why we are setting them to HIGH (off)
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);

  // Wait until serial port is open
  while(!Serial1) {
    ledErrorCode(1, 1, 1);
  }

  if(Serial1.available()) {
    Serial.println(F("Ready."));
    Serial.println(F("---------------"));
  }

  Serial.println(F("Booting..."));
  for(int i = 0; i < 10; i++) {
    Serial.print(F("."));
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
        // In addition to this, wait until we are mobile (moving at over 2kmph)
        // 5kmph for a reason: sometimes while obtaining a fix the speed is innacurately reported (0-4.5kmph) while stationary
        // TODO: A better way to find out if we have moved from our initial starting zone would be to set a geofence during setup(), then proceed to loop(). Once we are outside of the geofence, start logging data.
        if(gps.date.year() != '2000' && gps.speed.kmph() >= 5) {
          String dataStr = ""; // data string for saving
          dataStr += String(gps.date.year()) + "-"+ 
                     String(gps.date.month()) + "-" + 
                     String(gps.date.day()) + ",";
          dataStr += String(gps.time.hour()) + ":" + 
                     String(gps.time.minute()) + ":" + 
                     String(gps.time.second()) + "." + 
                     String(gps.time.centisecond()) + ",";
          dataStr += String(gps.location.lat(), 7) + ","; // latitude
          dataStr += String(gps.location.lng(), 7) + ","; // longitude
          dataStr += String(gps.speed.kmph()); // speed
          
          // Print location data to serial
          displayInfo(); 

          // Flash green LED
          digitalWrite(GREEN, LOW);
          writeFile(dataStr);
          delay(200);

          // Run loop every 2 seconds
          delay(2000);
        } else {
          // Not mobile
          digitalWrite(GREEN, HIGH);
          digitalWrite(BLUE, LOW);
          delay(300);
          digitalWrite(BLUE, HIGH);
          delay(100);
        }
      } else {
        if(!gps.location.isValid()) {
          Serial.println(F("Waiting on location sync."));
          ledErrorCode(1, 3, 1);
        }
        if(!gps.date.isValid()) {
          Serial.println(F("Waiting on date sync."));
          ledErrorCode(1, 4, 1);
        }
        if(!gps.time.isValid()) {
          Serial.println(F("Waiting on time sync."));
          ledErrorCode(1, 5, 1);
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
        Serial.print(F("Location: "));
        Serial.print(gps.location.lat(), 7);
        Serial.print(F(","));
        Serial.println(gps.location.lng(), 7);
      } else {
        Serial.println(F("Location: Not Available"));
      }

      Serial.print(F("Date: "));
      if (gps.date.isValid() && gps.date.year() != '2000') {
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.println(gps.date.year());
      } else {
        Serial.println(F("Not Available"));
      }

      Serial.print(F("Time: "));
      if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.println(gps.time.centisecond());
      } else {
        Serial.println(F("Not Available"));
      }
}

void initSD() {
  Serial.println(F("\nInitializing SD card..."));
 
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    Serial.println(F("Error initializing SD card..."));
    // Error - show LED error code 12
    ledErrorCode(1, 2, 10);
    sd.initErrorHalt(&Serial);
    return;
  } else {
    Serial.println(F("SD Card initialized."));
  }
}

// Function for writing data to SD card
void writeFile(String WriteData) {

  /** 
  Create a file, with a filename of today's date
  Data is stored in GMT time, so a post-conversion will have to be done.

  CSV Header is as follows:
  Date [yyyy-mm-dd], Time [HH:MM:SS.ZZ], Latitude [deg], Longitude [deg], Speed [km/hr]
  **/

  Serial.print(F("Writing the following data to file: "));
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
      Serial.println(F("Unable to write file to SD card.")); // print error if SD card issue
      ledErrorCode(2, 1, 5);
    }
  } else {
    Serial.println(F("Waiting for GPS date sync..."));
    ledErrorCode(1, 4, 1);
  }
}

// Print a GPS dump for debugging
void GPSDump() {
  Serial.print(F("Latitude: "));
  Serial.println(gps.location.lat(), 7); // Latitude in degrees (double)
  Serial.print(F("Longitude: "));
  Serial.println(gps.location.lng(), 7); // Longitude in degrees (double)
  Serial.print(F("Date: "));
  Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
  Serial.print(F("Speed (mps): "));
  Serial.println(gps.speed.mps()); // Speed in meters per second (double)
  Serial.print(F("Speed (km/hr): "));
  Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
}

// Flash LED sequence times to indicate an error code
void ledErrorCode(int longFlash, int shortFlash, int cycles) {
  // Turn off green LED (just in case it was already on)
  digitalWrite(GREEN, HIGH);
  for(int counter = 0; counter < cycles; counter++) {  
    for(int i = 0; i < longFlash; i++) {
        digitalWrite(BLUE, HIGH);
        digitalWrite(RED, LOW);
        delay(1000);  
        digitalWrite(RED, HIGH);
        delay(250);
        for(int i = 0; i < shortFlash; i++) {
          digitalWrite(BLUE, LOW);
          delay(250);
          digitalWrite(BLUE, HIGH);
          delay(250);
          digitalWrite(BLUE, LOW);
        }
    }
  }

  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
}
