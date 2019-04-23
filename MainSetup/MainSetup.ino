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

// changeable settings
boolean TURNON=true;
int BLACK[]={300,280,240};
#define MAXSPEED 60
#define TURNSPEED 80
#define CYCLERATE 3
//settings end

int noblack=0;
//motor code
int MotorDir[]={DIR_M1,DIR_M2,DIR_M3,DIR_M4};
int MotorStr[]={PWM_M1,PWM_M2,PWM_M3,PWM_M4};
void initMotors(){// default setup code for the multimoto motor shield
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
    analogWrite(MotorStr[0],TURNSPEED+10);
  analogWrite(MotorStr[2],MAXSPEED+10);
  analogWrite(MotorStr[1],MAXSPEED+10);
  analogWrite(MotorStr[3],TURNSPEED+10);
}

void turnLeft(){
 digitalWrite(MotorDir[0],0);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],0);
    analogWrite(MotorStr[0],MAXSPEED+10);
  analogWrite(MotorStr[2],TURNSPEED+10);
  analogWrite(MotorStr[1],TURNSPEED+10);
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
//  cycle++;
    readPixy(190);
//  if(cycle==CYCLERATE){//reads the ultrasonic sensor in certain increments to not lag out the robot
//    cycle=0;
//     frontPing=readPing(1);
//
//  }
  for(int count=0;count<3;count++){
  scales[count]=convertColor(analogRead(scaleAddress[count]), count); 
  }
//  output();
}
int convertColor(int rawValue, int index){//distinguishes between white (0) and black (2) using preset values 
  if(rawValue>BLACK[index]){
    return 2;
  }
  return 0;
}
void output(){//writes the values of the grayscale to the console
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
void readPixy(int bound){//reads all values of the pixy
  
  green=0;
  int left=0;
  int right=0;
  uint16_t blocks;
  char buf[32];
  blocks=pixy.getBlocks();//calls the blocks from the pixy
  if(blocks){
    for(int count=0;count<blocks;count++){//since the only signature we set was green, we don't need to check the sigID
      int x=pixy.blocks[count].x;
      int y=pixy.blocks[count].y;
      int w=pixy.blocks[count].width;
      int h=pixy.blocks[count].height;
      if(y+(h/2)>bound){//checks if the green is lower down in the screen
        if(x<160){//checks if it is left or right
//          Serial.println("LEFT FOUND");
          left=2;
        }
        else{
//          Serial.println("RIGHT FOUND");
          right=1;
        }
      }
      
    }  
  }
  green=left+right;
}

//pixy code end

//decision code
void flip(){//the 360 code turn the robot around
  green=0;
  Serial.println("360");
      turnRight();
      delay(1000);
      for(int count=0;count<200;count++){//refreshes the grayscale values
        delay(1);
        refreshColor();
      }
      while(scales[1]!=2){//will keep on turning until it sees black again
        refreshColor();
      }
      moveForward();
      delay(100);
}
int validateTurn(){//checks each turn to make sure the turn is a REAL green (not one after the black)
  //this is based on the fact that if the robot turns on a green AFTER the black line, it will have to do a turn of over 90 degrees
  green=0;
  delay(500);
      for(int count=0;count<200;count++){
        refreshColor();
        delay(1);
      }
//      refreshColor();
      for(int count=0;count<50;count++){
        refreshColor();
         if(scales[1]==2){
          moveForward();
          return 1;//returns 1 if the black line was sensed again
        }
        delay(10);
      }
      return 0;//returns 0 if the black line wasn't found in a 90 degree turn
}
void avoidObstacle(){//the overall manuever for the obstacle avoidance
  moveBackward();
  delay(700);
  turnLeft();
  delay(400);
  moveForward();
  delay(1700);
  turnRight();
  delay(900);
  moveForward();
  refreshColor();
  while(scales[1]!=2){
  refreshColor();
  }
}
void lineFollow(){
refreshColor();
if(scales[1]==2){//goes FORWARD on default
  noblack=0;
//    if(frontPing<12){//if the front ping senses an obstacle close to it, activate avoid
//      avoidObstacle();
//      frontPing=1000;
//      return;
//    }
    if(green==1){//if the pixy sees green on the right, turn right
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
    else if(green==2){//if the pixy sees green on the left, turn left
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
      turnRight();
      while(scales[1]!=2&&scales[2]!=2){
        refreshColor();
      }
      }
    }
    else if(green==3){//if there are two greens, turn around
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
    noblack=0;
    Serial.println("TILT LEFT");
    turnLeft();
    while(scales[0]!=0&&scales[1]!=2){
      Serial.println("TILT LEFT");
      refreshColor();
    }
    moveForward();
  }
  else if(scales[2]==2){//if middle isn't black and right is black/green, turn right
    noblack=0;
    Serial.println("TILT RIGHT");
    turnRight();
    while(scales[2]!=0&&scales[1]!=2){
      Serial.println("TILT RIGHT");
      refreshColor();
    }
    moveForward();
  }
  else{//if nothing sees black, it might be in rescue so start counting up 
    noblack++;
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
int activate=0;
int loops=4;
void loop(){
  if(noblack>2000){
    Serial.println("TICK");
    stopMotor();
    activate=1; 
    delay(2000);
    noblack=0;
  }
  
if(activate==1){
  refreshColor();
  if(scales[1]==2){
    activate=0;
    delay(2000);
    
  }
  int right=readPing(2);
  int mid=readPing(1);
  
  if(mid<15){
    if(right>40){
      loops--;
      if(loops==0){
        turnRight();
        delay(800);
      }
      else{
        turnLeft();
        delay(800);
      }
    }
    turnLeft(); 
    delay(100); 
  }
  else{
    moveForward();
  }
}
else{
  lineFollow();

}
 }

