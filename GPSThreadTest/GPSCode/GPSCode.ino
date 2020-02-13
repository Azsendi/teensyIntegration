#include <TinyGPS++.h>
#include "TeensyThreads.h"

#define Xbee Serial3
#define GPS_SERIAL Serial2

TinyGPSPlus GPS;

void setup() {
  // put your setup code here, to run once:
  Xbee.begin(9600);
  Serial.begin(9600);

  threads.addThread(GPSThread);
  GPS_SERIAL.begin(9600, SERIAL_8N1);

}

byte b;
void GPSThread() {
  while (1) {
    if (GPS_SERIAL.available()) {
      b = GPS_SERIAL.read();
      GPS.encode(b);
      //Serial.write(b);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(GPS.location.lat(),10);
  Serial.print("\t");
  Serial.print(GPS.location.lng(),10);
  Serial.print("\t");
  Serial.println(GPS.altitude.feet(),10);
}
