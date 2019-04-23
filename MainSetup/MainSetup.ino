#include <SPI.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <PixyI2C.h>
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
#define MAXSPEED 60
#define TURNSPEED 80
#define TURNDELAY 300
#define CYCLERATE 5
// changeable settings
boolean TURNON=true;
boolean PINGACTIVATE=true;
int BLACK[]={340,240,230};
//settings end

//motor code
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
void stopMotor(){
  digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],1);
    analogWrite(MotorStr[0],0);
  analogWrite(MotorStr[2],0);
  analogWrite(MotorStr[1],0);
  analogWrite(MotorStr[3],0);
}
void moveForward(){
 if(TURNON){
 digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],1);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],MAXSPEED+30);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],MAXSPEED+30);
 }
}
void moveBackward(){
  if(TURNON){
 digitalWrite(MotorDir[0],0);
 digitalWrite(MotorDir[1],0);
 digitalWrite(MotorDir[2],0);
 digitalWrite(MotorDir[3],0);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],MAXSPEED);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],MAXSPEED);
 }
}
void turnRight(){
 digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],0);
 digitalWrite(MotorDir[2],0);
 digitalWrite(MotorDir[3],1);
    analogWrite(MotorStr[0],TURNSPEED);
  analogWrite(MotorStr[2],MAXSPEED+10);
  analogWrite(MotorStr[1],MAXSPEED+10);
  analogWrite(MotorStr[3],TURNSPEED);
}

void turnLeft(){
 digitalWrite(MotorDir[0],0);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],0);
    analogWrite(MotorStr[0],MAXSPEED+10);
  analogWrite(MotorStr[2],TURNSPEED);
  analogWrite(MotorStr[1],TURNSPEED);
  analogWrite(MotorStr[3],MAXSPEED+10);
}
//motor code end
//ultrasonic code
int pingAddress[]={32,30,28};
int readPing(int index){
  long duration;
  pinMode(pingAddress[index],OUTPUT);
  digitalWrite(pingAddress[index], LOW);
  delayMicroseconds(2);
  digitalWrite(pingAddress[index], HIGH);
  delayMicroseconds(5);
  digitalWrite(pingAddress[index], LOW);
  pinMode(pingAddress[index], INPUT);
  duration = pulseIn(pingAddress[index], HIGH);
  // convert the time into a distance
  int cm = microsecondsToCentimeters(duration);
  return cm;
}
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
//ultrasonic code end
//grayscale code
int scaleAddress[]={13,14,15};
int scales[]={0,0,0};
int frontPing=10000;
int cycle=0;
void refreshColor(){
  cycle++;
    readPixy(190);
  if(cycle==CYCLERATE){
    cycle=0;
     frontPing=readPing(1);
//    delay(2000);
  }
  for(int count=0;count<3;count++){
  scales[count]=convertColor(analogRead(scaleAddress[count]), count); 
  }
  output();
}
int convertColor(int rawValue, int index){
  if(rawValue>BLACK[index]){
    return 2;
  }
  return 0;
}
void output(){
  for(int count=0;count<3;count++){
    Serial.print(scales[count]);
    Serial.print(" ");
  }
  Serial.println();
}
//grayscale code end
//Pixy Code
PixyI2C pixy;
int green=0;
void readPixy(int bound){
  
  green=0;
  int left=0;
  int right=0;
  uint16_t blocks;
  char buf[32];
  blocks=pixy.getBlocks();
  if(blocks){
    for(int count=0;count<blocks;count++){
      int x=pixy.blocks[count].x;
      int y=pixy.blocks[count].y;
      int w=pixy.blocks[count].width;
      int h=pixy.blocks[count].height;
//      Serial.println(x);
//      Serial.println(y);
//      Serial.println(w);
//      Serial.println(h);
//      Serial.println("--------------------------");
      if(y+(h/2)>bound){
        if(x<160){
//          Serial.println("LEFT FOUND");
          left=2;
        }
        else{
//          Serial.println("RIGHT FOUND");
          right=1;
        }
      }
      
    }  
//    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  }
  green=left+right;
}

//pixy code end

//decision code
void flip(){
  green=0;
  BLACK[0]=300;
  BLACK[2]=300;
  for(int count=0;count<15;count++){
  refreshColor();
  if(scales[0]==2&&scales[2]==2){
    BLACK[0]=200;
    BLACK[2]=200;
    return;
  }
    delay(3);
  }
  BLACK[0]=200;
  BLACK[2]=200;
  Serial.println("360");
      turnRight();
      delay(1800);
      for(int count=0;count<200;count++){
        delay(1);
        refreshColor();
      }
      while(scales[1]!=2&&scales[2]!=2){
        refreshColor();
      }
           
}
int validateTurn(){
  green=0;
  delay(200);
      for(int count=0;count<200;count++){
        refreshColor();
        delay(1);
      }
//      refreshColor();
      for(int count=0;count<60;count++){
        refreshColor();
         if(scales[1]==2){
          moveForward();
          return 1;
        }
        delay(10);
      }
      return 0;
}
void avoidObstacle(){
//  int left=readPing(0);
  moveBackward();
  delay(700);
  turnLeft();
  delay(500);
  moveForward();
  delay(2500);
  turnRight();
  delay(1500);
  moveForward();
  refreshColor();
  while(scales[1]!=2){
  refreshColor();
  }
}
void lineFollow(){
refreshColor();

if(scales[1]==2){//goes FORWARD on default
    if(frontPing<5){
      avoidObstacle();
      frontPing=1000;
      return;
    }
    if(green==1){
      moveBackward();
      delay(200);
      //at any point of the go forward, if the code sees two green instead, it will do a 360
      stopMotor();
      for(int count=0;count<100;count++){
      readPixy(100);
      if(green==3){
       flip();
      return;
      }
      delay(10);
      }
      Serial.println("TURN RIGHT");
      moveForward();
      delay(700);
      turnRight();
      if(validateTurn()==1){
      }
      else{
      turnLeft();
        while(scales[1]!=2&&scales[0]!=2){  
        refreshColor();
        }  
      }
    }
    else if(green==2){
      moveBackward();
      delay(200);
      //at any point of the go forward, if the code sees two green instead, it will do a 360
      stopMotor();
      for(int count=0;count<100;count++){
      readPixy(100);
      if(green==3){
        flip();
      return;
      }
      delay(10);
      }
      Serial.println("TURN LEFT");
      moveForward();
      delay(700);
      turnLeft();
      if(validateTurn()==1){
        Serial.println("HAHAHAHAHA");
        delay(1000);
      }
      else{
//      Serial.println("OOF----------------------------------------");
//      delay(500);
      turnRight();
      while(scales[1]!=2&&scales[2]!=2){
        refreshColor();
      }
      }
    }
    else if(green==3){
      flip();
    }
    else{
    Serial.println("FORWARD");
    moveForward();
    }    
}
else{
  //if middle isn't black and left is black/green, turn left
  if(scales[0]==2){
    
    Serial.println("TILT LEFT");
    turnLeft();
    while(scales[0]!=0&&scales[1]!=2){
      Serial.println("TILT LEFT");
      refreshColor();
    }
    moveForward();
  }
  else if(scales[2]==2){//if middle isn't black and left is black/green, turn right
    Serial.println("TILT RIGHT");
    turnRight();
    while(scales[2]!=0&&scales[1]!=2){
      Serial.println("TILT RIGHT");
      refreshColor();
    }
    moveForward();
  }
  else{
    Serial.println("LOST");
  }
    
}
}

//decision code end
void setup(){
  initMotors();
  pixy.init();
  Serial.begin(250000);
}


void loop(){
//  frontPing=readPing(1);
//  Serial.println(frontPing);
lineFollow();
//delay(1);  
 }

