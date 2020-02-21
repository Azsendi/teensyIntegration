//************ Imports
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <MS5611.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//********** Variables
File DATALOG;
const int chipSelect = BUILTIN_SDCARD;
uint32_t sdClock = millis();
uint32_t timer = millis();
uint32_t alitimeterClock = millis();
float maxHeight = 0, currentSpeed = 0;
unsigned long previousTime, deltaT;
float previousAltitude, relativeAltitude, referenceAltitude;
boolean LDA = false, velocityCheck = false, apogeeCheck = false, fireDrogue = false, fireMain = false;
float total = 0;
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;

//********** Definitions
#define Xbee Serial3
#define GPS_SERIAL Serial2
#define OUTPUT_READABLE_QUATERNION
#define INTERRUPT_PIN 2


//********** Sensor definitions
TinyGPSPlus GPS;
MS5611 ms5611;
MPU6050 mpu;

//**************** MPU Setup
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Xbee.begin(9600); //might change depending on Xbee setup
  Serial.print("Initializing SD card...");
  ms5611.begin(MS5611_ULTRA_HIGH_RES);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;//Note: This only works on a teensy, and will freeze the code if there is no SD inserted
  }
  Serial.println("initialization done.");

  DATALOG = SD.open("DATALOG.csv", FILE_WRITE); // Opens the datalog file
  DATALOG.println("\n\n\n\nNew Test");//Separates Old data from New
  DATALOG.println("Time,\tW,\tX,\tY,\tZ"); // First row of new data to define the columns
  DATALOG.close();
  sdClock = millis();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //Setup for GPS
  GPS_SERIAL.begin(9600, SERIAL_8N1);

  //Setting up the altitude filter
  referenceAltitude = ms5611.getAltitude(ms5611.readPressure());
  while (readIndex < numReadings) {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = ms5611.getAltitude(ms5611.readPressure());
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
  }
  readIndex = 0;
  referenceAltitude = total / numReadings;

}

void loop() {
  // put your main code here, to run repeatedly:
  barometerCalculations();
  Serial.println("\nBaro");
  gpsData();
  Serial.println("\nGPS");
  imuCalculations();
  Serial.println("\nIMU");
  printInfo();
  Serial.println("\nData");
  XBeeData();
  Serial.println("\nXBee");
//  sdWriting();
}

void barometerCalculations() { //Copied from previous code. Works for sure if it's just barometer only
  float baroAlt = ms5611.getAltitude(ms5611.readPressure());
  relativeAltitude = baroAlt - referenceAltitude;
  deltaT = millis() - previousTime;
  currentSpeed = 1000 * abs(relativeAltitude - previousAltitude) / (deltaT);
  if (!LDA && relativeAltitude > 10) {
    LDA = true;
  }

  //Giving errors right now, will fix later
  if (!LDA && abs(relativeAltitude) < 1) {

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = baroAlt;
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

void imuCalculations() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read a packet from FIFO
    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }
  }

  #ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
  #endif
}

void gpsData() { // just checking to see if the gps is working
  byte b = GPS_SERIAL.read();
  GPS.encode(b);
  Serial.write(b);
  delay(100);
}

void printInfo() {
  Serial.print(GPS.location.lat(), 10);
  Serial.print("\t");
  Serial.print(GPS.location.lng(), 10);
  Serial.print("\t");
  Serial.print(GPS.altitude.feet(), 10);
  Serial.print("\t");
  Serial.print(deltaT); // currently around 100 ms
  Serial.print("\t");
  Serial.print(relativeAltitude - previousAltitude);
  Serial.print("\t");
  Serial.print(ms5611.getAltitude(ms5611.readPressure()));
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
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.print(q.z);
  Serial.println();
  //delay(100);
}

void XBeeData() {
  Xbee.print("data");
}

void sdWriting() {
  if (millis() - sdClock > 100) {
    sdClock = millis();
    DATALOG = SD.open("DATALOG.csv", FILE_WRITE);
    //Time,\tW,\tX,\tY,\tZ
    DATALOG.print(millis());
    DATALOG.print(",\t");
    DATALOG.print(q.w);
    DATALOG.print(",\t");
    DATALOG.print(q.z);
    DATALOG.print(",\t");
    DATALOG.print(q.y);
    DATALOG.print(",\t");
    DATALOG.println(q.z);
    DATALOG.close();
  }
}
