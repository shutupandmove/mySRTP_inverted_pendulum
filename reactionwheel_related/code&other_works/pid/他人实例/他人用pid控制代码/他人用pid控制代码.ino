#include<Wire.h>
#include <Servo.h>
#include <PID_v1.h>

int value;
int value2 = 0;
// set values you need to zero

int sensVal;           // for raw sensor values 
float filterVal = 0.001;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;     // this holds the last loop value just use a unique variable for every different sensor that needs smoothing

float smoothedVal2;   // this would be the buffer value for another sensor if you needed to smooth two different sensors - not used in this sketch


int i, j;  

Servo firstESC;
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int goal = 300;
double Input;
double Output;
double Setpoint = 14000;
int Kp = 5;
int Ki = 0.5;
int Kd = 1;
PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);  
int neutral = 1300;
int fullForward = 2600;
int fullReverse = 0;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(0,255);
  Serial.begin(9600);         
  // initialize all the readings to 0:
  Serial.println("Calibration procedure for Mamba ESC.");
  Serial.println("Turn on ESC.");
  Serial.println("Press any key to continue.");
  while (!Serial.available());
  Serial.read();
  
  firstESC.writeMicroseconds(0);
  Serial.println("Starting Calibration.");
  delay(5000);
  firstESC.writeMicroseconds(2600);
  Serial.println("Writing Full Throttle.");
  delay(5000);
  firstESC.writeMicroseconds(0);
  Serial.println("Writing Full Reverse.");
  delay(5000);
  firstESC.writeMicroseconds(1300);
  Serial.println("Writing Neutral.");
  delay(10000);
  Serial.println("Calibration Complete.");


  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  
}
void loop(){
  Input = abs(smooth(readGyro(),filterVal,smoothedVal));
//  if(Input < 0){
//    balancePID.SetControllerDirection(REVERSE);
//  }
//  else{
//    balancePID.SetControllerDirection(DIRECT);
//  }
//  if(Input > 15000){
//    balancePID.SetControllerDirection(REVERSE);
//  }
//  if(Input < 15000){
//    balancePID.SetControllerDirection(DIRECT);
//  }
  Serial.print("Average Value (X): ");Serial.println(Input);
  balancePID.Compute();
  if(abs(Input) > 15000){
    value = (Output*5.3);
      motorSpeed(value);
  }
  if(abs(Input) < 15000){
    value = (Output*5.3)+1249;
      motorSpeed(value);
  }
  Serial.println(value);
 


}

int motorSpeed(int newValue){
  firstESC.writeMicroseconds(newValue);
//  if(Serial.available()){ 
//      value = Serial.parseInt();
//  }
    //motor has been calibraed, below values correspond to .writeMicroseconds values
    //1300 == neutral
    //2600 == full forward
    //0 == full reverse

}

int averageValue(int GyX){
    total= total - readings[index];
    readings[index] = GyX;
    total= total + readings[index];     
    index = index + 1;                    
    if (index >= numReadings){              
      index = 0;                          
    }
    average = total / numReadings;  
    //Serial.print("Average Value (X): ");Serial.println(average); 
   return average;   
}

int readGyro(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print(" | AcY = "); Serial.print(AcY);
//  Serial.print(" | AcZ = "); Serial.print(AcZ);
//  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.println(" | GyX = "); Serial.print(GyX);
//  Serial.print(" | GyY = "); Serial.print(GyY);
//  Serial.print(" | GyZ = "); Serial.println(GyZ);
  
  //delay(333);
  if(AcY < 0){
    return ((AcX^2)+(AcY^2))^(1/2);
  }
  if(AcY > 0){
    return -((AcX^2)+(AcY^2))^(1/2);
  }
}

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

