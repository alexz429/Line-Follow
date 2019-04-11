#define PINNUM 13
int pins[]={13,14,15};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}
int small=1000;
int big=0;
String com="de";
int mode=0;
void loop() {
    if(Serial.available()){
      com=Serial.readString();
      if(com.equals("scan")){
        mode=1;
        com="de";
      }
      
      if(com.equals("color")){
        mode=2;
        com="de";
      }
      if(com.equals("back")){
        mode=0;
        com="de";
      }
      
    }
    if(mode==1){
      change();
    }
    if(mode==2){
      returnColor();
    }
  // put your main code here, to run repeatedly:
}

void returnColor(){
  for(int count=0;count<3;count++){
    int val=analogRead(pins[count]);
  Serial.print(val);
  Serial.print(" ");
  
    
  }
  Serial.println();
  
}
void change(){
  
  int val=analogRead(PINNUM);
  if(small>val){
    small=val;
  }
  if(big<val){
    big=val;
  }
  if(com.equals("reset")){
    small=1000;
    big=0;
    Serial.println("reset");
    delay(1000);
    com="de";
    Serial.read();
  }
  Serial.print(small);
  Serial.print(" ");
  Serial.println(big);
}

