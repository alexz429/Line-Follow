
#include <SPI.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
  Adafruit_DCMotor *M2 = AFMS.getMotor(2);
  Adafruit_DCMotor *M3 = AFMS.getMotor(3);
  Adafruit_DCMotor *M4 = AFMS.getMotor(4);
void initMotors(){//setup input output pins for motors
  
  
}
void setup() {
  // put your setup code here, to run once:
  initMotors();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  M1->setSpeed(200);
  M1->run(FORWARD);
}
