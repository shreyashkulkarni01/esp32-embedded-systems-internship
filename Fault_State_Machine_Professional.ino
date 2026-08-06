/*
 Fault_State_Machine_Professional.ino
 Skeleton professional example for ESP32 + I2C LCD
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

#define ADC1_PIN 34
#define ADC2_PIN 35
#define ADC3_PIN 32

#define GREEN_LED 2
#define YELLOW_LED 4
#define RED_LED 5
#define RELAY_PIN 19
#define BUZZER_PIN 18

enum State { NORMAL, DEGRADED, FAILSAFE, SHUTDOWN };
State currentState = NORMAL, previousState = NORMAL;

unsigned long lastLog=0;
int lastRaw1=-1,lastRaw2=-1,lastRaw3=-1;
int freezeCount=0;

float adcToVoltage(int raw){
  return (raw/4095.0f)*3.3f;
}

void logTransition(const char *fault){
  Serial.printf("{\"time\":%lu,\"previous\":%d,\"current\":%d,\"fault\":\"%s\"}\n",
                millis(),previousState,currentState,fault);
}

void enterState(State s,const char *fault){
  if(s==currentState) return;
  previousState=currentState;
  currentState=s;
  logTransition(fault);
}

void updateOutputs(){
  digitalWrite(GREEN_LED,currentState==NORMAL);
  digitalWrite(YELLOW_LED,currentState==DEGRADED);
  digitalWrite(RED_LED,currentState==FAILSAFE || currentState==SHUTDOWN);

  if(currentState==NORMAL){
    digitalWrite(RELAY_PIN,HIGH);
    noTone(BUZZER_PIN);
  }else if(currentState==DEGRADED){
    digitalWrite(RELAY_PIN,HIGH);
  }else{
    digitalWrite(RELAY_PIN,LOW);
    tone(BUZZER_PIN,1000);
  }
}

void setup(){
  Serial.begin(115200);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(RELAY_PIN,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.print("Fault Monitor");
}

void loop(){

  int r1=analogRead(ADC1_PIN);
  int r2=analogRead(ADC2_PIN);
  int r3=analogRead(ADC3_PIN);

  float v1=adcToVoltage(r1);
  float v2=adcToVoltage(r2);
  float v3=adcToVoltage(r3);

  if(r1==lastRaw1 && r2==lastRaw2 && r3==lastRaw3)
    freezeCount++;
  else
    freezeCount=0;

  lastRaw1=r1;
  lastRaw2=r2;
  lastRaw3=r3;

  bool frozen = freezeCount>50;

  bool relayMismatch=false; // simulation placeholder

  if(frozen)
    enterState(FAILSAFE,"ADC_FROZEN");
  else if(relayMismatch)
    enterState(DEGRADED,"RELAY_MISMATCH");
  else
    enterState(NORMAL,"RECOVERED");

  updateOutputs();

  lcd.setCursor(0,0);
  lcd.print("S:");
  lcd.print((int)currentState);
  lcd.print("      ");

  lcd.setCursor(0,1);
  lcd.print(v1,2);
  lcd.print(" ");
  lcd.print(v2,2);

  delay(100);
}
