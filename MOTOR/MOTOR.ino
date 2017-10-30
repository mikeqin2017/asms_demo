#include <RBD_Timer.h>
#include<SPI.h>

RBD::Timer timer;

#define SLEEP 5
#define STEP 3
#define RESET 8
#define DIR 4
#define BIN1 6
#define BIN2 7

void motor_pwm(unsigned int delaytime);  //motor driver
void read_data(void);  //deal with serial data 
unsigned int read_data(byte add); //read data from register
void write_data(byte add,unsigned int data);  //write data to register
void DRV_init(void);  //RRV8711 inital 
void StepStartStop(void);

unsigned char nSleep,nStep,Dir;
unsigned int targetSpeed,stopSpeed,startSpeed,accelRate,nowSpeed;
unsigned int delayTime,addTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);      //Serial inital
  SPI.begin();               //SPI inital
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  digitalWrite(SS,LOW);
  
  DRV_init();
}

void loop() {
    // put your main code here, to run repeatedly:
    motor_pwm(delayTime); 
    StepStartStop();
    if (Serial.available() > 0) {
      read_data();
    }
}
void StepStartStop(){
  if(timer.onRestart()) {
    if(nStep){
       nowSpeed=nowSpeed+accelRate/1000; 
       delayTime=10000/(nowSpeed/100)-22;
      
       if(nowSpeed>=targetSpeed){
        delayTime=10000/(targetSpeed/100)-22;
        timer.stop();
      }
    }
    else{
        nowSpeed=nowSpeed-accelRate/1000; 
        delayTime=(10000/(nowSpeed/100)-22);
      if(nowSpeed<=stopSpeed){
        nowSpeed=0;
        delayTime=0;
        timer.stop();
      }    
    }
  }
}
void read_data(){
  unsigned char data[10];
  unsigned char i;
  for(i=0;i<10;i++){
    if (Serial.available() > 0) {
      data[i]=Serial.read();
      //Serial.println(data[i],HEX);   
    } 
    else{
      break;
    }
    delay(2);
  } Serial.flush();
        
    if(data[0]==0x00){        //write Register
      write_data(data[1],(data[2]<<8)|data[3]);
    }
    else if(data[0]==0x01){  //read Resister
      unsigned int ReData=read_data(data[1]);
      Serial.print(ReData);
    }
    else if(data[0]==0x02){   //change the frequncy of the motor  Step
      nStep=0x01&(~nStep);
      targetSpeed=data[1]*256+data[2];
      startSpeed=data[3]*256+data[4];
      stopSpeed=data[5]*256+data[6];
      accelRate=data[7]*256+data[8];
      if(nStep){
      nowSpeed=startSpeed;
      }
      timer.setTimeout(1);
      timer.restart();
    }
    else if(data[0]==0x03){   //enable or disable the motor
      nSleep=0x01&(~nSleep);
      if(nSleep){
        digitalWrite(SLEEP,HIGH);
      }
      else{
        digitalWrite(SLEEP,LOW);
      }
    }
    else if(data[0]==0x04){    
     Dir=0x01&(~Dir);
     if(Dir){
      digitalWrite(DIR,HIGH);
     }else{
      digitalWrite(DIR,LOW);
     }
    }
    else if(data[0]==0x05){
      digitalWrite(RESET,HIGH);
      delay(1);
      digitalWrite(RESET,LOW);
    }
    //Serial.println(nStep);
}


void motor_pwm(unsigned int delaytime){
  if(delaytime>12){
    digitalWrite(STEP,HIGH);
    delayMicroseconds(12);
    digitalWrite(STEP,LOW);
    delayMicroseconds(delaytime); 
  }
}
void write_data(byte add,unsigned int data){
  unsigned char low,high;
  low=(unsigned char)(data&0x00ff);
  high=(add<<4)|(data>>8);   
  digitalWrite(SS,HIGH);
  SPI.transfer(high);
  SPI.transfer(low);
  digitalWrite(SS,LOW); 
}

unsigned int read_data(byte add){
  unsigned int low,high;
  digitalWrite(SS,HIGH);
  high=SPI.transfer(((0x08|add)<<4));
  low=SPI.transfer(0);
  digitalWrite(SS,LOW);
  return (high<<8)|low;
}

void DRV_init(){
  unsigned int initData[7]={0x0f21,0x017b,0x0028,0x0096,0x0514,0x083c,0x00f0};
  unsigned char i;
  pinMode(SLEEP,OUTPUT);  //hardware part
  digitalWrite(SLEEP,LOW);
  pinMode(STEP,OUTPUT);
  digitalWrite(STEP,LOW);  
  pinMode(DIR,OUTPUT);
  digitalWrite(DIR,LOW);  
  pinMode(RESET,OUTPUT);
  digitalWrite(RESET,LOW);
  pinMode(BIN1,OUTPUT);
  digitalWrite(BIN1,LOW);
  pinMode(BIN2,OUTPUT);
  digitalWrite(BIN2,LOW);
  
  for(i=0;i<7;i++){          //software part
    write_data(i,initData[i]);  
    delay(2);
  }
}

