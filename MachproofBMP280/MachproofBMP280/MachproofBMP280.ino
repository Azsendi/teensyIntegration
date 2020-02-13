//Libraries
#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include <MS5611.h>
//#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include "TeensyThreads.h"

//Definitions
#define Xbee Serial3
#define GPS_SERIAL Serial2
//#define GPSECHO false

MS5611 ms5611;
//Adafruit_GPS GPS(&GPSSerial);
TinyGPSPlus GPS;

//Variables
float maxHeight = 0;
unsigned long previousTime, deltaT;
float previousAltitude, relativeAltitude, referenceAltitude;
boolean LDA = false, velocityCheck = false, apogeeCheck = false, fireDrogue = false, fireMain = false;
float total = 0;
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
uint32_t timer = millis();

float altitude() {
  return ms5611.getAltitude(ms5611.readPressure()); /* Adjusted to local forecast! */
}

void setup() {

  Xbee.begin(9600);
  Serial.begin(9600);
  Wire.begin();

  // Initialize MS5611
  Serial.println("Initialize MS5611");

  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while (!ms5611.begin(MS5611_ULTRA_HIGH_RES))
  {
    Serial.println("Could not find a valid MS5611, check wiring!");
    delay(500);
  }

  referenceAltitude = altitude();
  while (readIndex < numReadings) {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = altitude();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
  }
  readIndex = 0;
  referenceAltitude = total / numReadings;

  threads.addThread(gpsThread);
GPS_SERIAL.begin(9600, SERIAL_8N1);
  
}

//Initialize all the variables in the Loop
uint32_t sdClock = millis();
uint32_t alitimeterClock = millis();

byte b;
void gpsThread() {
  while (1) {
    if (GPS_SERIAL.available()) {
      b = GPS_SERIAL.read();
      GPS.encode(b);
      //Serial.write(b);
    }
  }
}

void barometer() {
  //Insert Machproof Code
  relativeAltitude = altitude() - referenceAltitude;
  deltaT = millis() - previousTime;
  float currentSpeed = 1000 * abs(relativeAltitude - previousAltitude) / (deltaT); //Caution: Might break due to units, check back later
  //Debugging outputs
  Serial.print(deltaT); // currently around 100 ms
  Serial.print("\t");
  Serial.print(relativeAltitude - previousAltitude);
  Serial.print("\t");
  Serial.print(altitude());
  Serial.print("\t");
  Serial.print(referenceAltitude);
  Serial.print("\t");
  Serial.print(relativeAltitude);
  Serial.print("\t");
  Serial.print(currentSpeed);
  Serial.print("\t");
  Serial.print(LDA);
  Serial.print("\t");
  Serial.print(velocityCheck);
  Serial.print("\t");
  Serial.print(apogeeCheck);
  Serial.print("\t");
  Serial.print(maxHeight);
  Serial.print("\t");
  Serial.print(fireDrogue);
  Serial.print("\t");
  Serial.print(fireMain);
  Serial.print("\t");
  Serial.print(abs(millis() - alitimeterClock));
  Serial.print("\t");

  if (!LDA && relativeAltitude > 10) {
    LDA = true;
  }

  //Giving errors right now, will fix later
  if (!LDA && abs(relativeAltitude) < 1) {

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = altitude();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }
    referenceAltitude = total / numReadings;
  }

  //Second Check Velocity
  if (LDA) {
    if (currentSpeed <= 30) {
      if (millis() - alitimeterClock > 1000 && !velocityCheck) {
        velocityCheck = true;
        alitimeterClock = millis();
      }
    } else {
      alitimeterClock = millis();
    }
  }

  //Third Check Apogee Delay & Drogue Firing
  if (velocityCheck && !fireDrogue) {
    if (relativeAltitude > maxHeight) {
      maxHeight = relativeAltitude;
      alitimeterClock = millis();
    } else {
      if (millis() - alitimeterClock > 1000 && abs(maxHeight - relativeAltitude) >= 5) {
        apogeeCheck = true;
        fireDrogue = true; //Placeholder Code, will replace later
      }
    }
  }

  //Main Firing
  if (fireDrogue) {
    if (relativeAltitude <= 15) {
      fireMain = true;// Placeholder code, will replae later
    }
  }

  previousAltitude = relativeAltitude;
  previousTime = millis();
}

void loop() {
  barometer(); // Calls the Barometer function
  Serial.println(GPS.location.lat(),6);
  Serial.println(GPS.location.lng(),6);
  if (millis() - sdClock >= 500) {
    sdClock = millis();
    XBeePrint();
  }
  Serial.println();
}

void XBeePrint() {
  String AltitudeData = (String) relativeAltitude + " m";
  Xbee.println(AltitudeData);
}
