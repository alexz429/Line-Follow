#include <SPI.h>
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
#define MAXSPEED 80
#define TURNSPEED 90
#define TURNDELAY 300
#define GREENCYCLE 300
#define ISGREEN 150

// changeable settings
boolean TURNON=true;

#define ARRAYLENGTH 50
int BLACK[]={300,180,300};
int GREEN[]={160,250,150};
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
void moveForward(){
 if(TURNON){
 digitalWrite(MotorDir[0],1);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],1);
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
  analogWrite(MotorStr[2],MAXSPEED);
  analogWrite(MotorStr[1],MAXSPEED);
  analogWrite(MotorStr[3],TURNSPEED);
 
}

void turnLeft(){
 digitalWrite(MotorDir[0],0);
 digitalWrite(MotorDir[1],1);
 digitalWrite(MotorDir[2],1);
 digitalWrite(MotorDir[3],0);
    analogWrite(MotorStr[0],MAXSPEED);
  analogWrite(MotorStr[2],TURNSPEED);
  analogWrite(MotorStr[1],TURNSPEED);
  analogWrite(MotorStr[3],MAXSPEED);
 
}
//motor code end

//grayscale code
int scaleAddress[]={14,13,15};
int pairings[3][ARRAYLENGTH];
int leftGreen[GREENCYCLE];
int rightGreen[GREENCYCLE];
int leftSum=0;
int rightSum=0;
int scales[]={0,0,0};
void resetGreen(){
  for(int count=0;count<GREENCYCLE;count++){
    leftGreen[count]=0;
    rightGreen[count]=0;
    leftSum=0;
    rightSum=0;
  }
}
void refreshColor(){
  for(int count=ARRAYLENGTH-1;count>0;count--){
    pairings[0][count]=pairings[0][count-1];
    pairings[1][count]=pairings[1][count-1];
    pairings[2][count]=pairings[2][count-1];
  }
  for(int count=0;count<3;count++){
    int next=analogRead(scaleAddress[count]);
    Serial.print(next);
    Serial.print(" ");
    pairings[count][0]=next;
    
  scales[count]=getReading(count); 
  }
  if(leftGreen[GREENCYCLE-1]==1){
    leftSum--;
  }
  if(rightGreen[GREENCYCLE-1]==1){
    rightSum--;
  }
    for(int count=GREENCYCLE-1;count>0;count--){
      leftGreen[count]=leftGreen[count-1];
      rightGreen[count]=rightGreen[count-1];
    }
  
  if(scales[0]==1){
    leftGreen[0]=1;
    leftSum++;
  }
  else{
    leftGreen[0]=0;
  }
  if(scales[2]==1){
    rightGreen[0]=1;
    rightSum++;
  }
  else{
    rightGreen[0]=0;
  }
  output();
}
int getReading(int index){
  int tally=0;
  for(int count=0;count<ARRAYLENGTH;count++){
    tally+=pairings[index][count];
  }
  tally=tally/ARRAYLENGTH;
  int out=convertColor(tally, index);
  
  return out;
}
int convertColor(int rawValue, int index){
  if(rawValue>BLACK[index]){
    
    return 2;
  }
  if(rawValue>GREEN[index]){
    return 1;
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

//decision code


void lineFollow(){
refreshColor();
if(scales[1]==2){
  if(leftSum>=ISGREEN&&rightSum>=ISGREEN){//if middle sees black and both are green, turn around
    Serial.println("TURN AROUND");
    turnRight();
    delay(4000);
    moveForward();
    resetGreen();
  }
  else if(leftSum>=ISGREEN){//if middle sees black and ONLY left is green, turnLeft
    Serial.println("TURN LEFT");
    moveForward();
    delay(TURNDELAY);
    turnLeft();
    delay(2000);
    resetGreen();
    
    moveForward();
  }
  else if(rightSum>=ISGREEN){//if middle sees black and ONLY right is green, turnRight
    Serial.println("TURN RIGHT");
    moveForward();
    delay(TURNDELAY);
    turnRight();
    delay(2000);
    resetGreen();
    moveForward();
  }
  else{//goes FORWARD on default
    Serial.println("FORWARD");
    moveForward();    
  
  }
}
else{
  //if middle isn't black and left is black/green, turn left
  if(scales[0]==2||scales[0]==1){
    Serial.println("TILT LEFT");
    turnLeft();
    while(scales[0]!=0&&scales[1]!=2){
      refreshColor();
    }
    moveForward();
  }
  else if(scales[2]==2||scales[2]==1){//if middle isn't black and left is black/green, turn right
    Serial.println("TILT RIGHT");
    turnRight();
    while(scales[2]!=0&&scales[1]!=2){
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
  Serial.begin(250000);
  resetGreen();
}


void loop(){
lineFollow();
delay(1);  
 }

