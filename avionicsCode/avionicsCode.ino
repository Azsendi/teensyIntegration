#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <MS5611.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

File DATALOG;
const int chipSelect = BUILTIN_SDCARD;
uint32_t sdClock = millis();
uint32_t timer = millis();

#define Xbee Serial3
#define GPS_SERIAL Serial2

TinyGPSPlus GPS;
MS5611 ms5611;
MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
