#include<SPI.h>

#define SLEEP 5
#define STEP 3
#define RESET 8
#define DIR 4
#define BIN1 6
#define BIN2 7

void motor_pwm(unsigned char Step);  //motor driver
void read_data(void);  //deal with serial data 
unsigned int read_data(byte add); //read data from register
void write_data(byte add,unsigned int data);  //write data to register
void DRV_init(void);

unsigned int delay_time1,delay_time2;
unsigned char nSleep,nStep;

void setup() {
  // put your setup code here, to run once:
  pinMode(SLEEP,OUTPUT);
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

  Serial.begin(9600);     
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  digitalWrite(SS,LOW);

  DRV_init();
}

void loop() {
  // put your main code here, to run repeatedly:
    motor_pwm(nStep);
    if (Serial.available() > 0) {
      read_data();
    }
    //read_data(0x00);
    //delay(1);
}

void read_data(){
  unsigned char data[10];
  unsigned char i;
  for(i=0;i<10;i++){
    if (Serial.available() > 0) {
      data[i]=Serial.read();
    }
    else{
      break;
    }
  }
    if(data[0]==0x00){        //write Register
      write_data(data[1],(data[2]<<8)|data[3]);
    }
    else if(data[0]==0x01){  //read Resister
      unsigned int ReData=read_data(data[1]);
    }
    else if(data[0]==0x02){   //change the frequncy of the motor
      
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
    else if(data[0]==0x04){    //Step
      nStep=0x01&(~nStep);
    }
}

void motor_pwm(unsigned char Step){
  if(Step==1){
    digitalWrite(STEP,HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP,LOW);
    delayMicroseconds(137);    
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
  for(i=0;i<7;i++){
    write_data(i,initData[i]);  
    delay(2);
  }
}

