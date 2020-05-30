#include <SoftwareSerial.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_PMTK.h"

#define chipSelect 10 // SD Pinout
#define GPS_ENABLE 1  // Set to 0 if you want live AQI but no GPS tracking
#define PIN        6  // LED Pinout
#define DEBUG      0  // Set to 0 for SD logging, 1 for serial output

/* LED Light Numbers */
#define NUMPIXELS 8
#define SYS       0
#define GPS       1
#define PMS       2
#define SDC       3
#define AQI       5

#define LIGHTLVL  255 // Brightness of LEDS scale from 1-255
#define GOODLED 0x392CAE
#define MODLED 0x832CAE 
#define WARNLED 0xC04AC6
#define HAZARDLED 0xF75EFF

SoftwareSerial gpsSerial(8, 7);
SoftwareSerial pmsSerial(2, 3);
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

File logfile;
char GPSBuff[100];
size_t len;

/* Data Structures */
// TODO Strip struct of data we won't use
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
/*******************/

struct pms5003data pmSensorData;

void setup() {
  /* LED Startup */
  pixels.begin();
  pixels.clear();
  /***************/
  
  /* SD card setup */
  pinMode(chipSelect, OUTPUT); //set pin 10 as output to SD
  if (!SD.begin(chipSelect)) { // Begin SD card on pin10
    pixels.setPixelColor(SDC, pixels.Color(LIGHTLVL, 0, 0)); // RED
  } else {
    char filename[15];
    strcpy(filename, "AQI_GPS.TXT");
    logfile = SD.open(filename, FILE_WRITE);
    if(!logfile) {
      pixels.setPixelColor(SDC, pixels.Color(LIGHTLVL, LIGHTLVL, 0)); // YELLOW
    } else {
      pixels.setPixelColor(SDC, pixels.Color(0, 0, LIGHTLVL)); // BLUE
    }
  }
  pixels.show();
  /****************/
  
  /* Debug output */
  if (DEBUG) Serial.begin(115200);
  Serial.println("hi");
  /****************/

  /* GPS Initialization */
  if (GPS_ENABLE) {
    if(!gpsSerial) {
      pixels.setPixelColor(GPS, pixels.Color(LIGHTLVL, 0, 0)); // RED
    } else {
      gpsSerial.begin(9600);
      gpsSerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY ); // data to collect
      gpsSerial.println(PMTK_SET_NMEA_UPDATE_1HZ); // data collection rate
      pixels.setPixelColor(GPS, pixels.Color(LIGHTLVL, LIGHTLVL, 0)); // YELLOW
    }
  }
  pixels.show();
  /**********************/

  /* PM2.5 Initialization */
  pmsSerial.begin(9600);
  /************************/
}

void loop() {
//   First Check pm sensor for data
  pmsSerial.listen();
  if (readPMSdata(&pmsSerial)) {
    if (GPS_ENABLE) {
      gpsSerial.listen();
      delay(1000);
      if (readGPSdata(&gpsSerial)) {
        if (valid()) {
          if (fix()) {
            pixels.setPixelColor(GPS, pixels.Color(0, 0, LIGHTLVL)); // BLUE
            pixels.show();
            if (DEBUG) {
              logToSerial();
            } else {
              logToSD();
            }
          } else {
            pixels.setPixelColor(GPS, pixels.Color(LIGHTLVL, LIGHTLVL, 0)); // YELLOW
            pixels.show();
          }
        }
      }
    } else {
      if (DEBUG) logToSerial();
      else logToSD();
    }
    setAQIled();
  }
}

bool readGPSdata(Stream *s)
{
  if (! s->available()) {
    return false;
  }  
  int i = 0;
  while ( s->peek() != 0xD ) {
    if (s->available()) {
      GPSBuff[i] = s->read();
      i++;
    } 
  }
  len = i;
  s->read();
  return true;
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
  
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&pmSensorData, (void *)buffer_u16, 30);
 
  if (sum != pmSensorData.checksum) {
    if (DEBUG) Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

bool valid()
{
  char check[6] = "$GPRMC";
  for(int i = 0; i <= 5; i++) {
    if(i+1 < len) {
      if(check[i] != GPSBuff[i]) return false;
    }
  }
  return true;
}

bool fix()
{
  char *p = GPSBuff;
  p = strchr(p, ',') + 1; // Skip to char after the next comma, this will be time, which we'll ignore.
  p = strchr(p, ',') + 1; // this is fix data
  p = strchr(p, ',') + 1; // Skip to char after the next comma, this will be time, which we'll ignore.
  p = strchr(p, ',') + 1; // this is fix data
  p = strchr(p, ',') + 1; // Skip to char after the next comma, this will be time, which we'll ignore.
  p = strchr(p, ',') + 1; // this is fix data
  if(*p == 'A') return true;
  return false;
}

void setAQIled()
{
  long int color = 0;
  int R, G, B;
  if(pmSensorData.pm25_standard <= 12) {
    color = GOODLED;
  } else if(pmSensorData.pm25_standard <= 35) {
    color = MODLED;
  } else if(pmSensorData.pm25_standard <= 55) {
    color = WARNLED;
  } else {
    color = HAZARDLED;
  }
  R = color>>16;
  G = color>>8 & 0xff;
  B = color & 0xff;
  pixels.show();
  if (!DEBUG) {
    for(int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(R, G, B));
      pixels.show();
    }
  } else {
    for(int i = 0; i < 8; i++) {
      pixels.setPixelColor(i, pixels.Color(R, G, B));
     // pixels.setPixelColor(AQI, pixels.Color(LIGHTLVL*R, LIGHTLVL*G + LIGHTLVL*O*0.5, 0));
      pixels.show();
    }
  }
}

void logToSD()
{
  logfile.print("PM 1.0: "); logfile.print(pmSensorData.pm10_standard);
  logfile.print("\t\tPM 2.5: "); logfile.print(pmSensorData.pm25_standard);
  logfile.print("\t\tPM 10: "); logfile.println(pmSensorData.pm100_standard);
  if (GPS_ENABLE) {
    logfile.write(GPSBuff, len);
    logfile.println();
  }
  logfile.flush();
}

void logToSerial()
{
  Serial.print("PM 1.0: "); Serial.print(pmSensorData.pm10_standard);
  Serial.print("\t\tPM 2.5: "); Serial.print(pmSensorData.pm25_standard);
  Serial.print("\t\tPM 10: "); Serial.println(pmSensorData.pm100_standard);
  Serial.flush();
  if (GPS_ENABLE) {
    Serial.write(GPSBuff, len);
    Serial.println();
  }
  Serial.flush();
}
