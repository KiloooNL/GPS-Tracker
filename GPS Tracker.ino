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
17: HDOP value too high
19: Speed too low
21: SD card write error
**/

#include <TinyGPSPlus.h> // Include TinyGPS+ library https://github.com/mikalhart/TinyGPSPlus
#include <Time.h>
#include <SPI.h>
#include <SdFat.h> // SDFat library https://github.com/greiman/SdFat

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
bool firstSave = true;
String name = "";

// Constant numbers
const float maxDistance = 20;     // Default: 20
const int minSpeed = 5;           // Default: 5
const int minSatellites = 4;      // Default: 4
const float maxHDOP = 5.0;        // Default: 5.0
const int maxLocationAge = 1800;  // Default: 1800

// Various timers to use instead of delay
unsigned long lastDisplayTime = 0;

// Use a macro for debugging, so that in production we can disable Serial.print functions freeing up memory.
// See this post: https://forum.arduino.cc/t/is-the-serial-println-output-buffered-when-not-connected-to-the-serial-console/966821/3 
#define DEBUG 0 // Set to 0 for production and 1 for debugging

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.write(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(bauds)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

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
  
  // Initialize SD Card
  initSD(); 
}

// Start the loop. Only work if GPS data is valid.
void loop() {
  while (Serial1.available()) {
    if(gps.encode(Serial1.read())) {
      if(gps.location.isValid() && gps.location.isUpdated() && gps.date.isValid() && gps.time.isValid()) {
        int numSatellites = gps.satellites.value();
        float hdop = gps.hdop.value() / 100;

        // On the first loop(), determine our starting longitude and latitude, then build a geofence around that location. Once we have moved outside that geofence, begin logging.
        if(numSatellites >= minSatellites && hdop < maxHDOP && gps.speed.isValid() && gps.speed.kmph() > minSpeed && gps.location.age() < maxLocationAge) {
          D_println(F("Good GPS fix"));

          // Turn off blue LED in case it was already on, then turn on green LED (success)
          digitalWrite(BLUE, HIGH);
          digitalWrite(GREEN, LOW);

          // Get 10 data points of longitude and latitude for accuracy, then use the average to determine our starting position
          if(initialLatitude == 0 && initialLongitude == 0) {
            D_println(F("Initialized and ready to set geofence."));
            writeLog("Initialized and ready to set geofence.");
            int loops = 0;
            while(loops < 10) {
              delay(250);
              initialLatitude += gps.location.lat();
              initialLongitude += gps.location.lng();
              loops++;
            }
            
            initialLatitude = (initialLatitude / 10.0);
            initialLongitude = (initialLongitude / 10.0);

            D_println(F("Set initial longitude and latitude: "));
            D_println(initialLatitude, 6);
            D_println(initialLongitude, 6);
            if(DEBUG) {
              writeLog("Set initial longitude and latitude.");
            }
          }

          if(outsideGeoFence == false) {          
            getGps(latitude, longitude);
            float distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);
            D_print(F("Current distance from starting location: ")); 
            D_print(distance);
            D_println(F(" meters"));

            if(distance > maxDistance) {
              outsideGeoFence = true;
              D_print(F("Now outside geofence. Distance: ")); 
              D_print(distance);
              D_println(F(" meters"));
              if(DEBUG) {
                writeLog("Now outside of geofence and will begin logging.");
              }
            }
          } else {
            if(millis() - lastDisplayTime >= 2000) { // Log data every 2 seconds
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
          if(numSatellites < minSatellites) {
            D_print(F("Waiting on satellites. Current satellites: "));
            D_println(numSatellites);
            ledErrorCode(1, 3, 1);
          } 
          if(hdop > maxHDOP) {
            D_println(F("HDOP value is too high. Current HDOP:"));
            D_println(hdop);
            ledErrorCode(1, 7, 1);
          }

          if(gps.speed.kmph() < minSpeed || !gps.speed.isValid()) {
            D_print(F("Speed is under 5kmph or not valid. Current speed: "));
            D_println(gps.speed.kmph());
            digitalWrite(GREEN, HIGH);
            digitalWrite(BLUE, LOW);
          }

          if(gps.location.age() > maxLocationAge) {
            D_print(F("Location age is over 1800ms. Current location age: "));
            D_println(gps.location.age());
            ledErrorCode(1, 8, 1);
          }
        }
      } else {
        if(!gps.location.isValid()) {
          D_println(F("Waiting on location sync."));
          ledErrorCode(1, 4, 1);
        }
        if(!gps.date.isValid()) {
          D_println(F("Waiting on date sync."));
          ledErrorCode(1, 5, 1);
        }
        if(!gps.time.isValid()) {
          D_println(F("Waiting on time sync."));
          ledErrorCode(1, 6, 1);
        }
      }
    }
  }
}

void initSD() {
  D_println(F("\nInitializing SD card..."));
 
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    D_println(F("Error initializing SD card..."));
    // Error - show LED error code 12
    ledErrorCode(1, 2, 10);
    sd.initErrorHalt(&Serial);
    return;
  } else {
    D_println(F("SD Card initialized."));
  }
}

// Function for writing data to SD card
void writeFile(String WriteData) {
  /** 
  Create a file, with a filename of today's date (format: YY-MM-DD_#)
  Data is stored in UTC time, so a post-conversion is performed when importing to a database.

  CSV Header is as follows:
  Date [yyyy-mm-dd], Time [HH:MM:SS.ZZ], Latitude [deg], Longitude [deg], Speed [km/hr], Age (fix), Satellites, HDOP
  **/
  if(firstSave) {
    String month = "";
    String day = "";

    if(gps.date.month() < 10) {
      month = "0" + String(gps.date.month()); 
    } else {
      month = String(gps.date.month());
    }

    if(gps.date.day() < 10) {
      day = "0" + String(gps.date.day());
    } else {
      day = String(gps.date.day());
    }

    name = String(gps.date.year()) + "-" + month + "-" + day + ".csv";

    D_print("File does not exist. Creating a new one. New file name: ");
    D_println(name);
  }
  
  if (file.open(name.c_str(), FILE_WRITE)) {
    if(firstSave) {
      // ########## is a header used to split the array of each trip in post-processing
      file.println("##########");
      firstSave = false;
    } else {
      file.println(WriteData); // write data to file
      D_print(F("Wrote the following data to file: "));
      D_println(WriteData);
    }
    file.close(); // close file before continuing 
  } else {
    delay(50); // prevents cluttering
    D_println(F("Unable to write file to SD card.")); // print error if SD card issue
    ledErrorCode(2, 1, 5);
  }
}

void writeLog(String WriteData) {
  String logFile = "log.txt";

  if(file.open(logFile.c_str(), FILE_WRITE)) {
    file.println(WriteData);
    file.close();
  } else {
    delay(50);
  }
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
    D_println("No GPS data is available");
    latitude = 0;
    longitude = 0;
  }
}

void ledErrorCode(int longFlash, int shortFlash, int cycles) {
  // Turn off all LEDs, in case they were already on
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);

  // If debugging is enabled, log the error code to log.txt
  if(DEBUG) {
    String logInfo = "";
    logInfo += "[";
    logInfo += gps.date.year();
    logInfo += "-";
    logInfo += gps.date.month();
    logInfo += "-";
    logInfo += gps.date.day();
    logInfo += " ";
    logInfo += gps.time.hour();
    logInfo += ":";
    logInfo += gps.time.minute();
    logInfo += "]";
    logInfo += " Error: ";
    logInfo += longFlash;
    logInfo += shortFlash;
    writeLog(logInfo);
  }

  for(int counter = 0; counter < cycles; counter++) {  
    for(int i = 0; i < longFlash; i++) {
        digitalWrite(BLUE, HIGH);
        digitalWrite(RED, LOW);
        unsigned long startTime = millis(); // Get start time
        while (millis() - startTime < 1000) { // Wait 1 second
          // Do nothing
        }
        digitalWrite(RED, HIGH);
        startTime = millis(); // Get start time
        while (millis() - startTime < 200) { // Wait 200ms
          // Do nothing
        }
        for(int i = 0; i < shortFlash; i++) {
          digitalWrite(BLUE, LOW);
          startTime = millis(); // Get start time
          while (millis() - startTime < 200) { // Wait 200ms
            // Do nothing
          }
          digitalWrite(BLUE, HIGH);
          startTime = millis(); // Get start time
          while (millis() - startTime < 200) { // Wait 200ms
            // Do nothing
          }
          digitalWrite(BLUE, LOW);
        }
    }
  }

  // Turn off red and blue LEDs
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
}
