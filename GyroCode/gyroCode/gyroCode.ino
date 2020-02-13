#include <TeensyThreads.h>
#include <SD.h>
#include <SD_t3.h>
#include <TinyGPS++.h>
//#include "gyro.h"
#include <MS5611.h>

#define GPS_SERIAL Serial2
#define XBEE_SERIAL Serial3

#define NEW_GPS_INFO_PACKET "GPSINFO,"
#define NEW_GPS_SAT_COUNT_PACKET "GPSSAT,"
#define NEW_ATM_PACKET "ATM,"
#define NEW_GYRO_PACKET "GYRODATA"
#define NEW_ACCEL_PACKET "ACCEL"



TinyGPSPlus gps;
MS5611 barro;

double zeroPressure = 0;
double startTimeMillis = 0;


uint8_t numSats = -1;

//115200


void setup() {

  threads.addThread(gpsThread);
  //Serial.begin(115200);
  GPS_SERIAL.begin(9600, SERIAL_8N1);
  XBEE_SERIAL.begin(9600, SERIAL_8N1);
  //XBEE_SERIAL.begin(9600);
  Serial.begin(9600);
  
  MPU6050_init();

  startTimeMillis = millis();
  barro.begin(MS5611_ULTRA_HIGH_RES);
}


byte b;
void gpsThread() {
  while (1) {
    if (GPS_SERIAL.available()) {
      b = GPS_SERIAL.read();
      gps.encode(b);
      //Serial.write(b);
    }
  }
}

void sendGpsInfoIfChanged() {
  if (gps.location.isUpdated()) {
    XBEE_SERIAL.print(NEW_GPS_INFO_PACKET);
    XBEE_SERIAL.print(gps.location.lat(), 6);
    XBEE_SERIAL.print(",");
    XBEE_SERIAL.print(gps.location.lng(), 6);
    XBEE_SERIAL.println();
    XBEE_SERIAL.print(",");
    XBEE_SERIAL.print(NEW_GPS_INFO_PACKET);
    XBEE_SERIAL.print(gps.date.value());
    XBEE_SERIAL.print(",");
    XBEE_SERIAL.print(gps.time.value());
    XBEE_SERIAL.print(",");
    XBEE_SERIAL.print(gps.speed.mps());
    XBEE_SERIAL.print(",");
    XBEE_SERIAL.print(gps.altitude.meters());
    XBEE_SERIAL.println();

  }

  if (gps.satellites.value() != numSats) {
    XBEE_SERIAL.print(NEW_GPS_SAT_COUNT_PACKET);
    XBEE_SERIAL.print(gps.satellites.value());
    XBEE_SERIAL.println();
    numSats = gps.satellites.value();
  }
}

void sendGyro() {
  accel_t_gyro_union accel_t_gyro = MPU6050_getVals();
  Serial.print("gyro x,y,z : ");
  Serial.print(accel_t_gyro.value.x_gyro, DEC);
  Serial.print(",");
  Serial.print(accel_t_gyro.value.y_gyro, DEC);
  Serial.print(",");
  Serial.print(accel_t_gyro.value.z_gyro, DEC);
  Serial.println();
}

void sendAccel() {
  accel_t_gyro_union accel_t_gyro = MPU6050_getVals();
  Serial.print(F("accel x,y,z: "));
  Serial.print(accel_t_gyro.value.x_accel, DEC);
  Serial.print(F(","));
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(F(","));
  Serial.print(accel_t_gyro.value.z_accel, DEC);
  Serial.println(F(""));

}

void sendAll() {
  Serial.print("O");
  Serial.print(gps.satellites.value());
  Serial.print("N");
  Serial.print(gps.location.lat(), 6);
  Serial.print("E");
  Serial.print(gps.location.lng(), 6);
  Serial.print("G");
  Serial.print(gps.date.value());
  Serial.print("T");
  Serial.print(gps.time.value());
  Serial.print("S");
  Serial.print(gps.speed.mps());
  Serial.print("F");
  Serial.print(gps.altitude.meters());
  Serial.print("BARO");
  Serial.print(barro.getAltitude(barro.readPressure()));
  Serial.println("");

}

int counter = 0;
int sdFlushCounter = 0;

void loop() {

  counter++;
  sdFlushCounter++;

  //Serial.println(millis());
  if (counter > 5000) {

    counter = 0;
    
    //sendAll(); 
    //sendGyro();
    sendAccel();
  }
}
