# bamboo
"Project Three Bamboo" installation code.

#include <Wire.h>                          
#include <SoftwareSerial.h>                
#include <Adafruit_MPR121.h>              

#define GlassTime          10000          
#define BaseFluidTime      16000          

#define AddNumber            2             
#define PumpNumber           6             
#define BambooTubeNumber     7             
#define DetectInterval       50            
#define EndDetectionInterval 10000         

#define ARDUINO_M_RX 2  
#define ARDUINO_M_TX 3  
SoftwareSerial mySerial(ARDUINO_M_RX, ARDUINO_M_TX);  

static int8_t Send_buf[8] = {0};    

// MP3module
#define SINGLE_CYCLE_ON 0X00
#define SINGLE_CYCLE_OFF 0X01
#define DEV_TF 0X02
#define CMD_PLAY_W_INDEX 0X03
#define CMD_SET_VOLUME 0X06
#define CMD_SEL_DEV 0X09
#define CMD_PLAY 0X0D
#define CMD_PAUSE 0X0E
#define CMD_SINGLE_CYCLE 0X19

bool BeginState = false;      
bool TriggerState = false;    

uint8_t i = 0;                 
uint16_t currtouched = 0;     
uint16_t lasttouched = 0;     
uint8_t RandomNumber = 0;     
uint8_t BambooTube_ID = 0;    
uint8_t lastRandomNumber = 0; 

uint8_t PumpPin[PumpNumber] = {4,5,6,7,8,9};                   
uint8_t SensorPin[BambooTubeNumber] = {14,15,16,17,18,19,11};  

unsigned long lastDetectTime = 0;   
unsigned long lastTriggerTime = 0;  


void sendCommand(int8_t command, int16_t dat) {
  delay(20);
  Send_buf[0] = 0x7e;  
  Send_buf[1] = 0xff;  
  Send_buf[2] = 0x06;  
  Send_buf[3] = command;
  Send_buf[4] = 0x00;                
  Send_buf[5] = (int8_t)(dat >> 8);  
  Send_buf[6] = (int8_t)(dat);       
  Send_buf[7] = 0xef;                
  for (uint8_t i = 0; i < 8; i++) {
    mySerial.write(Send_buf[i]);     
  }
}

// 传感器检测控制播放
void DetectPlay() {
  for(i = 0; i < BambooTubeNumber; i++){  
    if(digitalRead(SensorPin[i])){        
      if(!BeginState) {                   
        BeginState = true;                
        Serial.println("1,0,1,0");}       

      BambooTube_ID = i + 1;              
      TriggerState = true;                
      lastTriggerTime = millis();         
    }
  }
  if (TriggerState) {                              
    sendCommand(CMD_PLAY_W_INDEX, BambooTube_ID);  
    TriggerState = false;                         
  }
  delay(10);
}

// 
void MixLiquid(){
  digitalWrite(PumpPin[0], LOW);   
  delay(BaseFluidTime);
  digitalWrite(PumpPin[0], HIGH);    
  for(i = 0; i < AddNumber; i++){   
    RandomNumber = random(1,6);
    while(RandomNumber == lastRandomNumber)     
      RandomNumber = random(1,6);
    lastRandomNumber = RandomNumber;            
    digitalWrite(PumpPin[RandomNumber], LOW);  
    delay(GlassTime);
    digitalWrite(PumpPin[RandomNumber], HIGH);   
  }
}

//
void setup() {
  Serial.begin(9600);               
  mySerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);     

  for(i = 0; i < PumpNumber; i++)   
    pinMode(PumpPin[i], OUTPUT);

  for(i = 0; i < BambooTubeNumber; i++)   
    pinMode(SensorPin[i], INPUT_PULLUP);

  sendCommand(CMD_SEL_DEV, DEV_TF);  
  sendCommand(CMD_SET_VOLUME, 30);   
  sendCommand(CMD_PAUSE, 0X01);      

  for(i = 0; i < PumpNumber; i++)
    digitalWrite(PumpPin[i], HIGH);  

  digitalWrite(LED_BUILTIN, HIGH);  
  Serial.println("0,1,0,0");         
}

//
void loop() {
  if(millis() - lastDetectTime > DetectInterval){  
    DetectPlay();                                  
    lastDetectTime = millis();                      
  }

  if(BeginState){                                             
    if(millis() - lastTriggerTime > EndDetectionInterval){    
      digitalWrite(LED_BUILTIN, LOW);                         
      lastDetectTime = millis();                              
      BeginState = false;                                     
      Serial.println("2,0,0,1");                              
      MixLiquid();                                            
      digitalWrite(LED_BUILTIN, HIGH);                       
    }
  }
  
