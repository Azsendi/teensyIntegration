
#include <Wire.h>
#include <MS5611.h> 

MS5611 ms5611;
//Altimeter Variables
float referencePressure;
const int ledPin = 7;
int armingCOUNT = 0;
int firingCOUNT = 7;
float state1 = 0;
int FiringSequence = 0;

//MPU6050 Variables
const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;

void setup() 
{
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);

  // Initialize MS5611 sensor
  Serial.println("Initialize MS5611 Sensor");

  while(!ms5611.begin())
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // Get reference pressure for relative altitude
  referencePressure = ms5611.readPressure();

  Serial.print("Reference Pressure is:");
  Serial.println(ms5611.getAltitude(referencePressure));
  delay(1000);

  // Initialize MPU6050 Sensor
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}

void loop()
{
  /*
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  Serial.print("AccX = "); Serial.print(AccX);
  Serial.print(" || AccY = "); Serial.print(AccY);
  Serial.print(" || AccZ = "); Serial.print(AccZ);
  Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);
  Serial.print(" || GyroX = "); Serial.print(GyroX);
  Serial.print(" || GyroY = "); Serial.print(GyroY);
  Serial.print(" || GyroZ = "); Serial.println(GyroZ);
  delay(100);
  */
  
  // Turns on and off a random LED using pin 7
 
  
  // Read raw values
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  double realTemperature = ms5611.readTemperature();
  double realTemps = realTemperature*9/5+32;
  long realPressure = ms5611.readPressure();

  // Calculate altitude
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);

  float state2 = relativeAltitude;
  Serial.println("--");

  Serial.print(", Temperature = ");
  Serial.print(realTemps);
  Serial.println (" *F");

  //Serial.print(" rawPressure = ");
  //Serial.print(rawPressure);
  Serial.print(", realPressure = ");
  Serial.print(realPressure);
  Serial.println(" Pa");

  Serial.print(" absoluteAltitude = ");
  Serial.print(absoluteAltitude);
  Serial.print(" m, relativeAltitude = ");
  Serial.print(relativeAltitude);    
  Serial.println(" m");
  Serial.println(state2);
  Serial.println(state1);

  //Counting for the first check
  if(FiringSequence == 0) {
    // make sure its state2 > state1 for final code
    if(state2 > state1){
      armingCOUNT++;
    } else {
      armingCOUNT = 0;
    }
  }

  //first check
  //make sure it is relativeAltitude >= 200 (m) for final code
  if(armingCOUNT >= 7 && relativeAltitude >= 100 && FiringSequence == 0){
    FiringSequence = 1;
  }

  //Countdown for the second check
  if (FiringSequence == 1) {
    //make sure it is state1 > state2 for final code
    if (state1 > state2){
      firingCOUNT--;
    } else{
      firingCOUNT = 7;
    }
    digitalWrite(ledPin,HIGH);
    delay(125);
    digitalWrite(ledPin,LOW);
  }

  //Second Check
  if (firingCOUNT <= 0 && FiringSequence == 1) {
    FiringSequence = 2;
  }
  
  if (FiringSequence == 2){
    digitalWrite(ledPin,HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(5, HIGH);
  }
  
 
  Serial.println(firingCOUNT);

  Serial.print("Arming Count ");
  Serial.println(armingCOUNT);
  Serial.print("FiringCount ");
  Serial.println(firingCOUNT);
  Serial.print("Diff between Last and this: ");
  Serial.println(state2 - state1);
  delay(125);
 
  state1 = state2; 
  
}
