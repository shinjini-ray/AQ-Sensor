#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_PMTK.h"
#include <bluefruit.h>

#define MANUFACTURER_ID   0x004C 

#define PIN        6  // LED Pinout
#define DEBUG      0  // Set to 0 for phone logging, 1 for serial output

/* LED Light Numbers */
#define NUMPIXELS 8
#define SYS       0
#define PMS       2
#define AQI       5

#define LIGHTLVL  255 // Brightness of LEDS scale from 1-255
#define GOODLED 0x392CAE
#define MODLED 0x832CAE 
#define WARNLED 0xC04AC6
#define HAZARDLED 0xF75EFF

#define DEVICENAME "AirDavisPM2.5"

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// A valid Beacon packet consists of the following information:
// UUID, Major, Minor, RSSI @ 1M

BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

uint8_t writer = 1;

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
  
  /* PM2.5 Initialization */
  Serial1.begin(9600);
  while ( !Serial1 ) delay(10);
  /************************/
  
  /* Debug output */
  if (DEBUG) {
    Serial.begin(115200);
    while ( !Serial ) delay(10); 
  }
  /****************/

  /*Bluetooth*/
  Bluefruit.begin();
//  Bluefruit.setEventCallback(connect_callback);
  Bluefruit.setName(DEVICENAME);
  
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();
 
  // Configure and Start BLE Uart Service
  bleuart.begin();
 
  // Start BLE Battery Service
 // blebas.begin();
//  blebas.update(100);
  startAdv();
  /***********/

  pixels.setPixelColor(0, pixels.Color(200, 200, 200));
  pixels.show();

  /**********************/

}

void loop() {
//   First Check pm sensor for data
  logToPhone();
  if (readPMSdata(&Serial1)) {
    delay(1000);
    if (DEBUG) {
      logToSerial();
    } else {
      logToPhone();
    }
    setAQIled();
  }
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
     // pixels.setPixelColor(AQI, pixels.Color(LIGHTLVL*R, LIGHTLVL*G + LIGHTLVL*O*0.5, 0));
      pixels.show();
  }
}

void logToPhone()
{
  uint8_t pm25 = pmSensorData.pm25_standard;
  bleuart.write( writer );
}

void logToSerial()
{
  Serial.print("PM 1.0: "); Serial.print(pmSensorData.pm10_standard);
  Serial.print("\t\tPM 2.5: "); Serial.print(pmSensorData.pm25_standard);
  Serial.print("\t\tPM 10: "); Serial.println(pmSensorData.pm100_standard);
  Serial.flush();
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}
