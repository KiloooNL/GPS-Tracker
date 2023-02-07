/***
Simple GPS Tracker
https://github.com/KiloooNL/GPS-Tracker

See https://github.com/KiloooNL/GPS-Tracker/blob/main/README.md for credits, hardware and libraries.

Error codes are shown by the red and blue LED and are as follows
1 long red flash = 10 unit
1 short blue flash = 1 unit
For example: 1 long, 2 short = error code 12

11: Serial error (No communication from GPS unit)
12: SD Card error (No card, wrong chip select pin, or wiring error)
13: Waiting on satellites (over 3 satellites needed)
14: Waiting for location sync
15: Waiting on date sync
16: Waiting on time sync
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

// Set up our geofence for initial GPS location and zone
float initialLatitude = 0;
float initialLongitude = 0;
float latitude, longitude;
void getGps(float& latitude, float& longitude);
bool outsideGeoFence = false;

// Size of geofence (in meters)
const float maxDistance = 20;

// Create a timer to track
unsigned long lastDisplayTime = 0;

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
    ledErrorCode(1, 1, 2);
  }

  Serial.println(F("Booting..."));
  for(int i = 0; i < 10; i++) {
    Serial.print(F("."));
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Initialize SD Card
  initSD(); 
}

// Start the loop. Only work if GPS data is valid.
void loop() {
  while (Serial1.available()) {
    if(gps.encode(Serial1.read())) {
      if(gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) { // Run accuracyCheck() which validates if our GPS data is useable
        // On the first loop(), determine our starting longitude and latitude, then build a geofence around that location. Once we have moved outside that geofence, begin logging.
        if(gps.date.year() != '2000' && gps.location.isUpdated() && accuracyCheck()) {
          digitalWrite(BLUE, HIGH);
          // Flash green LED
          digitalWrite(GREEN, LOW);

          // Get 10 data points of longitude and latitude for accuracy, then use the average to determine our starting position
          if(initialLatitude == 0 && initialLongitude == 0) {
            Serial.println(F("Initialized and ready to set geofence."));
            int loops = 0;
            while(loops < 10) {
                delay(250); // Wait 250ms between location updates to provide more accuracy on early GPS fix
                initialLatitude += gps.location.lat();
                initialLongitude += gps.location.lng();
                loops++;
            }
            
            initialLatitude = (initialLatitude / 10.0);
            initialLongitude = (initialLongitude / 10.0);

            Serial.println(F("Set initial longitude and latitude: "));
            Serial.println(initialLatitude, 6);
            Serial.println(initialLongitude, 6);
          }

          if(outsideGeoFence == false && accuracyCheck()) {          
            getGps(latitude, longitude);
            float distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);
            Serial.print(F("Current distance from starting location: ")); 
            Serial.print(distance);
            Serial.println(F(" meters"));

            if(distance > maxDistance) {
              outsideGeoFence = true;
              Serial.print(F("Now outside geofence. Distance: ")); 
              Serial.print(distance);
              Serial.println(F(" meters"));
            }
          } else {
            if(accuracyCheck() && (millis() - lastDisplayTime >= 2000)) { // Check if the time is updated, this should prevent logging a duplicate data point
              // We're outside of the geofence. Start logging!
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
              dataStr += String(gps.speed.kmph()) + ","; // speed
              dataStr += String(gps.location.age()) + ","; // age
              dataStr += String(gps.satellites.value()) + ","; // Number of satellites in use
              dataStr += String(gps.hdop.value()); // Horizontal Dim. of Precision

              writeFile(dataStr);
              lastDisplayTime = millis();
            }
          }
        } else {
          if(gps.speed.kmph() < 5) {
            // Not mobile - show blue LED
            digitalWrite(GREEN, HIGH);
            digitalWrite(BLUE, LOW);
            delay(1000);
          }
        }
      } else {
        if(gps.satellites.value() < 3) {
          Serial.print(F("Waiting on satellites. Current satellites: "));
          Serial.println(gps.satellites.value());
          ledErrorCode(1, 3, 1);
        } else if(!gps.location.isValid()) {
          Serial.println(F("Waiting on location sync."));
          ledErrorCode(1, 4, 1);
        }
        if(!gps.date.isValid()) {
          Serial.println(F("Waiting on date sync."));
          ledErrorCode(1, 5, 1);
        }
        if(!gps.time.isValid()) {
          Serial.println(F("Waiting on time sync."));
          ledErrorCode(1, 6, 1);
        }
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
  Data is stored in UTC time, so a post-conversion will have to be done.

  CSV Header is as follows:
  Date [yyyy-mm-dd], Time [HH:MM:SS.ZZ], Latitude [deg], Longitude [deg], Speed [km/hr], Age (fix), Satellites, HDOP
  **/

  // Make sure our current time is valid, otherwise we are writing useless data
  // Write file to SD card - format: YYMMDD
  // TODO: Change filename to a better format, as SDFat library supports > 8 char filenames (see: https://forum.arduino.cc/t/sdfat-long-file-name-length-limit-is-255-chars-including-extension/546132)
  if(gps.time.isValid() && gps.date.year() != '2000') {
    Serial.print(F("Writing the following data to file: "));
    Serial.println(WriteData);
    
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
  Serial.print(F("Time: "));
  Serial.println(gps.time.value());
  Serial.print(F("Speed (km/hr): "));
  Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
  Serial.print(F("Location age: "));
  Serial.println(gps.location.age());
  Serial.print(F("Satellites: "));
  Serial.println(gps.satellites.value()); // Satellites
  Serial.print(F("HDOP: "));
  Serial.println(gps.hdop.value()); // Satellites
}

// Calculate distance between two points
float getDistance(float flat1, float flon1, float flat2, float flon2) {

  // Variables
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(flat1);
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2*atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));
  
  dist_calc *= 6371000.0; // Converting to meters

  return dist_calc;
}

void getGps(float& latitude, float& longitude)
{
  // Can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;){
    while (Serial1.available()){
      if (gps.encode(Serial1.read())){
        newData = true;
        break;
      }
    }
  }
  
  if (newData) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    newData = false;
  } else {
    Serial.println("No GPS data is available");
    latitude = 0;
    longitude = 0;
  }
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
        delay(200);
        for(int i = 0; i < shortFlash; i++) {
          digitalWrite(BLUE, LOW);
          delay(200);
          digitalWrite(BLUE, HIGH);
          delay(200);
          digitalWrite(BLUE, LOW);
        }
    }
  }


  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
}

bool accuracyCheck() {
  if(gps.time.isUpdated()
    && gps.speed.kmph() > 2 
    && gps.location.age() < 1500 
    && gps.satellites.value() > 2 
    && (gps.hdop.value() / 100) < 4) {
     Serial.println(F("Accuraccy check passed."));
     return true;
  } else {
    Serial.println(F("Accuracy check did not pass."));
    if(!gps.time.isUpdated()) {
      Serial.println(F("Reason: Time is not updated."));
    }
    if(gps.speed.kmph() < 2) {
      Serial.println(F("Reason: Speed is under 2kmph."));
    }
    if(gps.location.age() > 1500) {
      Serial.println(F("Reason: Location age is over 1500ms."));
    }
    if(gps.satellites.value() < 3) {
      Serial.println(F("Reason: Under 2 satellites available."));
    }
    if((gps.hdop.value() / 100) > 4) {
      Serial.println(F("Reason: HDOP value is too high."));
    }

    GPSDump();
    delay(100);
    return false;
  }
}
