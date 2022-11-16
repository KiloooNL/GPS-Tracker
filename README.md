# GPS-Tracker
Simple GPS tracker using Arduino hardware

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
During boot it will create a geofence of 20 meters - this is our 'starting zone'.
Once we are outside the starting zone (i.e. we are mobile), it will start logging the data.
It will update every 2-3 seconds.
