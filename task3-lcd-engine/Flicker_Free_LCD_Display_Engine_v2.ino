#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

#define POT1 34
#define POT2 35
#define POT3 32
#define BUTTON_PIN 23

const unsigned long REFRESH_INTERVAL=200;
const unsigned long PAGE_INTERVAL=3000;

enum Page{BATTERY,SYSTEM,TELEMETRY1,TELEMETRY2};
Page currentPage=BATTERY;

bool fault=false;
String oldL1="",oldL2="";
unsigned long lastRefresh=0,lastPage=0;

void drawLine(uint8_t row,String txt){
  while(txt.length()<16) txt+=" ";
  txt=txt.substring(0,16);
  String &old=(row==0)?oldL1:oldL2;
  for(int i=0;i<16;i++){
    if(old.length()<16 || old[i]!=txt[i]){
      lcd.setCursor(i,row);
      lcd.print(txt[i]);
    }
  }
  old=txt;
}

float readV(int pin){
  return analogRead(pin)*3.3/4095.0;
}

void showBattery(){
  float b1=readV(POT1),b2=readV(POT2),b3=readV(POT3);
  if(b1<1.0||b2<1.0||b3<1.0) fault=true;
  drawLine(0,"B1:"+String(b1,2)+" B2:"+String(b2,2));
  drawLine(1,"B3:"+String(b3,2));
}

void showSystem(){
  drawLine(0,"STATE:NORMAL");
  drawLine(1,"Relay:ON");
}

void showTelemetry1(){
  drawLine(0,"ADC1:"+String(analogRead(POT1)));
  drawLine(1,"ADC2:"+String(analogRead(POT2)));
}

void showTelemetry2(){
  drawLine(0,"ADC3:"+String(analogRead(POT3)));
  drawLine(1,"Telemetry Pg2");
}

void showFault(){
  drawLine(0,"*** FAULT ***");
  drawLine(1,"LOW BATTERY");
}

void setup(){
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  analogReadResolution(12);
  lcd.init();
  lcd.backlight();
  drawLine(0,"LCD Engine");
  drawLine(1,"Starting...");
  delay(1000);
  oldL1=""; oldL2="";
}

void loop(){
  if(digitalRead(BUTTON_PIN)==LOW) fault=false;
  unsigned long now=millis();

  if(!fault && now-lastPage>=PAGE_INTERVAL){
    currentPage=(Page)((currentPage+1)%4);
    lastPage=now;
  }

  if(now-lastRefresh>=REFRESH_INTERVAL){
    lastRefresh=now;
    if(fault){
      showFault();
    }else{
      switch(currentPage){
        case BATTERY: showBattery(); break;
        case SYSTEM: showSystem(); break;
        case TELEMETRY1: showTelemetry1(); break;
        case TELEMETRY2: showTelemetry2(); break;
      }
    }
  }
}
