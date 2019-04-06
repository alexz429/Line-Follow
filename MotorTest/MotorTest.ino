#include <SPI.h>
#include <Wire.h>             //Include the Wire Library
#include <I2Cdev.h>
#include <Wire.h>
#define SS_M4 14
#define SS_M3 13
#define SS_M2 12
#define SS_M1 11
// L9958 DIRection pins
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7
// L9958 PWM pins
#define PWM_M1 9
#define PWM_M2 10    // Timer1
#define PWM_M3 5
#define PWM_M4 6     // Timer0
#define ENABLE_MOTORS 8
#define MAXSPEED 50
int MotorDir[]={DIR_M1,DIR_M2,DIR_M3,DIR_M4};
int MotorStr[]={PWM_M1,PWM_M2,PWM_M3,PWM_M4};

void initMotors(){
  unsigned int configWord;
  Serial.println("Motor test!");
  pinMode(SS_M1, OUTPUT); digitalWrite(SS_M1, LOW);  // HIGH = not selected
  pinMode(SS_M2, OUTPUT); digitalWrite(SS_M2, LOW);
  pinMode(SS_M3, OUTPUT); digitalWrite(SS_M3, LOW);
  pinMode(SS_M4, OUTPUT); digitalWrite(SS_M4, LOW);

  // L9958 DIRection pins
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);

  // L9958 PWM pins
  pinMode(PWM_M1, OUTPUT);  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);  digitalWrite(PWM_M2, LOW);    // Timer1
  pinMode(PWM_M3, OUTPUT);  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);  digitalWrite(PWM_M4, LOW);    // Timer0

  // L9958 Enable for all 4 motors
  pinMode(ENABLE_MOTORS, OUTPUT); 
 digitalWrite(ENABLE_MOTORS, HIGH);  // HIGH = disabled
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE1);  // clock pol = low, phase = high

  // Motor 1
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M1, HIGH);
  // Motor 2
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  // Motor 3
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  // Motor 4
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);

  //Set initial actuator settings to pull at 0 speed for safety
  digitalWrite(ENABLE_MOTORS, LOW);// LOW = enabled  
}
void setup(){
  initMotors();
  
}
void moveForward(){
  
 digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],1);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],MAXSPEED);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],MAXSPEED);

}
void turnRight(){
 digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],0);
 digitalWrite(MotorDir[2],0);
 digitalWrite(MotorDir[3],1);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],MAXSPEED);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],MAXSPEED);
 
}

void turnLeft(){
 digitalWrite(MotorDir[0],0);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],0);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],MAXSPEED);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],MAXSPEED);
 
}
void loop(){
 moveForward();
 delay(2000);
 turnRight();
 delay(2000);
 turnLeft();
 delay(2000);
 }

