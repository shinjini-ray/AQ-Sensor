#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(8, 7);
SoftwareSerial pmsSerial(2, 3);

Adafruit_GPS GPS(&gpsSerial);

/* Data Structures */
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct gpsData {
  uint16_t checksum;
};


struct gpsData gpsSensorData;
struct pms5003data pmSensorData;

void setup() {
  /* Debug output */
  Serial.begin(115200);
  
  /* GPS Initialization */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // data to collect
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // data collection rate

  /* PM2.5 Initialization */
  pmsSerial.begin(9600);
}

void loop() {
  // First Check pm sensor for data
  if (readPMSdata(&pmsSerial)) { 
    // Data has been read into pmSensorData
    debugPMS();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  // put your main code here, to run repeatedly:
}

void debugPMS()
{
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(pmSensorData.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(pmSensorData.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(pmSensorData.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(pmSensorData.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(pmSensorData.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(pmSensorData.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(pmSensorData.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(pmSensorData.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(pmSensorData.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(pmSensorData.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(pmSensorData.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(pmSensorData.particles_100um);
    Serial.println("---------------------------------------");
}

bool readPMSdata(Stream *s) {
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
 
   //debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
  memcpy((void *)&pmSensorData, (void *)buffer_u16, 30);
 
  if (sum != pmSensorData.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  return true;
}
