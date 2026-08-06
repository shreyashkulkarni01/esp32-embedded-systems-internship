
/******************************************************
 Enterprise Blynk Analytics Dashboard
 ESP32 + Wokwi
******************************************************/

#define BLYNK_TEMPLATE_ID "TMPL32c1_mE9Q"
#define BLYNK_TEMPLATE_NAME "Enterprise Blynk Analytics"
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

LiquidCrystal_I2C lcd(0x27, 16, 2);

BlynkTimer timer;

//--------------- Pin Mapping ----------------//

#define CELL1_PIN 34
#define CELL2_PIN 35
#define CELL3_PIN 32
#define TEMP_PIN 33
#define SOC_PIN 39

#define RED_LED 2
#define ORANGE_LED 4
#define BLUE_LED 5
#define YELLOW_LED 16

#define BUZZER_PIN 18
#define RELAY_PIN 19
#define BUTTON_PIN 23

//--------------- Blynk Virtual Pins ----------------//

#define VP_SOC V0
#define VP_IMBALANCE V1
#define VP_TEMP V2
#define VP_STATE V3
#define VP_RISK V4
#define VP_HEALTH V5
#define VP_FAULTS V6
#define VP_UPTIME V7
#define VP_RECOMMEND V8
#define VP_SUMMARY V9

//--------------- FSM ----------------//

enum SystemState
{
  NORMAL,
  DEGRADED,
  FAILSAFE,
  SHUTDOWN
};

SystemState currentState = NORMAL;

//--------------- Variables ----------------//

float cellVoltage[3];

float temperature = 0;

float soc = 0;

float imbalance = 0;

int riskScore = 0;

int batteryHealth = 100;

int faultCount = 0;

unsigned long bootTime;

//--------------- Sensor Functions ----------------//

float readCellVoltage(int pin)
{
  int adc = analogRead(pin);

  return map(adc,0,4095,3000,4200)/1000.0;
}

float readTemperature()
{
  int adc = analogRead(TEMP_PIN);

  return map(adc,0,4095,20,80);
}

float readSOC()
{
  int adc = analogRead(SOC_PIN);

  return map(adc,0,4095,0,100);
}

//--------------- Read All Sensors ----------------//

void readSensors()
{
  cellVoltage[0]=readCellVoltage(CELL1_PIN);

  cellVoltage[1]=readCellVoltage(CELL2_PIN);

  cellVoltage[2]=readCellVoltage(CELL3_PIN);

  temperature=readTemperature();

  soc=readSOC();

  float maxV=max(cellVoltage[0],max(cellVoltage[1],cellVoltage[2]));

  float minV=min(cellVoltage[0],min(cellVoltage[1],cellVoltage[2]));

  imbalance=(maxV-minV)*1000;
}
/*****************************************************
                PART 2
      STATE MACHINE & HARDWARE CONTROL
*****************************************************/

//----------------------------------------------------
// Calculate Risk Score
//----------------------------------------------------

void calculateRisk()
{
  riskScore = 0;

  // Voltage Imbalance
  if (imbalance > 200)
    riskScore += 40;
  else if (imbalance > 100)
    riskScore += 20;

  // Temperature
  if (temperature > 60)
    riskScore += 30;
  else if (temperature > 45)
    riskScore += 15;

  // State of Charge
  if (soc < 20)
    riskScore += 30;
  else if (soc < 40)
    riskScore += 15;

  // Manual Fault Injection
  if (digitalRead(BUTTON_PIN) == LOW)
    riskScore = 100;

  riskScore = constrain(riskScore, 0, 100);

  batteryHealth = 100 - riskScore;
}

//----------------------------------------------------
// State Machine
//----------------------------------------------------

void updateStateMachine()
{
  if (riskScore < 25)
    currentState = NORMAL;

  else if (riskScore < 50)
    currentState = DEGRADED;

  else if (riskScore < 80)
    currentState = FAILSAFE;

  else
    currentState = SHUTDOWN;
}

//----------------------------------------------------
// LEDs + Relay + Buzzer
//----------------------------------------------------

void updateOutputs()
{
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, LOW);

  digitalWrite(BUZZER_PIN, LOW);

  switch (currentState)
  {
    case NORMAL:

      digitalWrite(BLUE_LED, HIGH);
      digitalWrite(RELAY_PIN, HIGH);

      break;

    case DEGRADED:

      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(RELAY_PIN, HIGH);

      break;

    case FAILSAFE:

      digitalWrite(ORANGE_LED, HIGH);
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);

      break;

    case SHUTDOWN:

      digitalWrite(RED_LED, HIGH);
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH);

      break;
  }
}

//----------------------------------------------------
// LCD Display
//----------------------------------------------------

void updateLCD()
{
  lcd.clear();

  lcd.setCursor(0, 0);

  switch (currentState)
  {
    case NORMAL:
      lcd.print("NORMAL");
      break;

    case DEGRADED:
      lcd.print("DEGRADED");
      break;

    case FAILSAFE:
      lcd.print("FAILSAFE");
      break;

    case SHUTDOWN:
      lcd.print("SHUTDOWN");
      break;
  }

  lcd.setCursor(0, 1);

  lcd.print("R:");
  lcd.print(riskScore);

  lcd.print("% ");

  lcd.print("SOC:");

  lcd.print((int)soc);

  lcd.print("%");
}

//----------------------------------------------------
// Main Control Function
//----------------------------------------------------

void controlSystem()
{
  readSensors();

  calculateRisk();

  updateStateMachine();

  updateOutputs();

  updateLCD();
}
/*****************************************************
                PART 3
      ENTERPRISE ANALYTICS ENGINE
*****************************************************/

//----------------------------------------------------
// Fault History
//----------------------------------------------------

String faultHistory[10];
int faultIndex = 0;

void logFault(String msg)
{
  faultHistory[faultIndex] = msg;
  faultIndex++;

  if (faultIndex >= 10)
    faultIndex = 0;

  faultCount++;
}

//----------------------------------------------------
// Recommendation Engine
//----------------------------------------------------

String recommendation = "";

void generateRecommendation()
{
  recommendation = "";

  if(currentState == NORMAL)
  {
    recommendation = "Battery Healthy";
  }

  if(currentState == DEGRADED)
  {
    recommendation = "Monitor Cell Imbalance";
  }

  if(currentState == FAILSAFE)
  {
    recommendation = "Perform Battery Balancing";
  }

  if(currentState == SHUTDOWN)
  {
    recommendation = "Replace Faulty Cell";
  }

  if(temperature > 60)
  {
    recommendation += " | Cooling Required";
  }

  if(soc < 20)
  {
    recommendation += " | Recharge Battery";
  }
}

//----------------------------------------------------
// Executive Summary
//----------------------------------------------------

String executiveSummary()
{
  String stateText;

  switch(currentState)
  {
    case NORMAL:
      stateText="NORMAL";
      break;

    case DEGRADED:
      stateText="DEGRADED";
      break;

    case FAILSAFE:
      stateText="FAILSAFE";
      break;

    case SHUTDOWN:
      stateText="SHUTDOWN";
      break;
  }

  String txt="";

  txt += "State:";
  txt += stateText;

  txt += " | Health:";
  txt += String(batteryHealth);
  txt += "%";

  txt += " | Risk:";
  txt += String(riskScore);

  txt += "%";

  txt += " | Faults:";
  txt += String(faultCount);

  return txt;
}

//----------------------------------------------------
// Uptime
//----------------------------------------------------

String getUptime()
{
  unsigned long sec=(millis()-bootTime)/1000;

  int hr=sec/3600;

  sec%=3600;

  int min=sec/60;

  sec%=60;

  return String(hr)+"h "+String(min)+"m "+String(sec)+"s";
}

//----------------------------------------------------
// Log State Changes
//----------------------------------------------------

SystemState previousState=NORMAL;

void monitorStateChange()
{
  if(previousState!=currentState)
  {
    String txt="";

    switch(currentState)
    {
      case NORMAL:
        txt="NORMAL";
        break;

      case DEGRADED:
        txt="DEGRADED";
        break;

      case FAILSAFE:
        txt="FAILSAFE";
        break;

      case SHUTDOWN:
        txt="SHUTDOWN";
        break;
    }

    logFault("State Changed -> "+txt);

    previousState=currentState;
  }
}

//----------------------------------------------------
// Update Blynk Dashboard
//----------------------------------------------------

void updateDashboard()
{
  Blynk.virtualWrite(V0,soc);

  Blynk.virtualWrite(V1,imbalance);

  Blynk.virtualWrite(V2,temperature);

  Blynk.virtualWrite(V4,riskScore);

  Blynk.virtualWrite(V5,batteryHealth);

  Blynk.virtualWrite(V6,faultCount);

  Blynk.virtualWrite(V7,getUptime());

  Blynk.virtualWrite(V8,recommendation);

  Blynk.virtualWrite(V9,executiveSummary());

  String stateText="";

  switch(currentState)
  {
    case NORMAL:
      stateText="NORMAL";
      break;

    case DEGRADED:
      stateText="DEGRADED";
      break;

    case FAILSAFE:
      stateText="FAILSAFE";
      break;

    case SHUTDOWN:
      stateText="SHUTDOWN";
      break;
  }

  Blynk.virtualWrite(V3,stateText);
}

//----------------------------------------------------
// Enterprise Analytics Task
//----------------------------------------------------

void analyticsTask()
{
  controlSystem();

  monitorStateChange();

  generateRecommendation();

  updateDashboard();
}
/*****************************************************
                PART 4
        SETUP() AND LOOP()
*****************************************************/

//----------------------------------------------------
// Initialize Hardware
//----------------------------------------------------

void initializePins()
{
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
  digitalWrite(RED_LED, LOW);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

//----------------------------------------------------
// LCD Startup Screen
//----------------------------------------------------

void startupScreen()
{
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Enterprise");

  lcd.setCursor(0,1);
  lcd.print("Battery BMS");

  delay(2000);

  lcd.clear();
}

//----------------------------------------------------
// Blynk Connected
//----------------------------------------------------

BLYNK_CONNECTED()
{
  Serial.println("Blynk Connected");

  Blynk.syncAll();
}

//----------------------------------------------------
// Setup
//----------------------------------------------------

void setup()
{
  Serial.begin(115200);

  bootTime = millis();

  initializePins();

  Wire.begin(21,22);

  lcd.init();

  lcd.backlight();

  startupScreen();

  WiFi.begin(ssid, pass);

  Serial.print("Connecting WiFi");

  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(500);

    Serial.print(".");
  }

  Serial.println();

  Serial.println("WiFi Connected");

  Blynk.config(BLYNK_AUTH_TOKEN);

  while(!Blynk.connect())
  {
    Serial.println("Connecting Blynk...");
    delay(1000);
  }

  Serial.println("Blynk Connected");

  timer.setInterval(2000L, analyticsTask);
}

//----------------------------------------------------
// Loop
//----------------------------------------------------

void loop()
{
  Blynk.run();

  timer.run();
}
/*****************************************************
                PART 5
        ENTERPRISE FEATURES
*****************************************************/

//----------------------------------------------------
// Trend Analysis
//----------------------------------------------------

const int TREND_SIZE = 20;

float imbalanceHistory[TREND_SIZE];
int trendIndex = 0;

void updateTrend()
{
  imbalanceHistory[trendIndex] = imbalance;

  trendIndex++;

  if(trendIndex >= TREND_SIZE)
    trendIndex = 0;
}

float averageImbalance()
{
  float sum = 0;

  for(int i=0;i<TREND_SIZE;i++)
    sum += imbalanceHistory[i];

  return sum / TREND_SIZE;
}

//----------------------------------------------------
// Composite Risk Score
//----------------------------------------------------

void calculateEnterpriseRisk()
{
  int imbalanceRisk = constrain(imbalance / 2, 0, 40);

  int tempRisk = 0;

  if(temperature > 60)
    tempRisk = 30;
  else if(temperature > 45)
    tempRisk = 15;

  int socRisk = 0;

  if(soc < 20)
    socRisk = 30;
  else if(soc < 40)
    socRisk = 15;

  riskScore = imbalanceRisk + tempRisk + socRisk;

  riskScore = constrain(riskScore,0,100);

  batteryHealth = 100 - riskScore;
}

//----------------------------------------------------
// Battery Balancing Detection
//----------------------------------------------------

bool balancingRequired()
{
  return imbalance > 120;
}

//----------------------------------------------------
// Critical Notification
//----------------------------------------------------

void sendAlerts()
{
  static bool sent = false;

  if(currentState == SHUTDOWN && !sent)
  {
    Blynk.logEvent("critical_fault",
                   "Battery entered SHUTDOWN state");

    sent = true;
  }

  if(currentState != SHUTDOWN)
    sent = false;
}

//----------------------------------------------------
// LCD Page Rotation
//----------------------------------------------------

byte lcdPage = 0;

void rotateLCD()
{
  lcd.clear();

  switch(lcdPage)
  {

    case 0:

      lcd.setCursor(0,0);
      lcd.print("State");

      lcd.setCursor(0,1);

      switch(currentState)
      {
        case NORMAL: lcd.print("NORMAL"); break;
        case DEGRADED: lcd.print("DEGRADED"); break;
        case FAILSAFE: lcd.print("FAILSAFE"); break;
        case SHUTDOWN: lcd.print("SHUTDOWN"); break;
      }

      break;

    case 1:

      lcd.setCursor(0,0);
      lcd.print("SOC:");

      lcd.print((int)soc);

      lcd.print("%");

      lcd.setCursor(0,1);

      lcd.print("Temp:");

      lcd.print(temperature);

      lcd.print("C");

      break;

    case 2:

      lcd.setCursor(0,0);
      lcd.print("Health:");

      lcd.print(batteryHealth);

      lcd.print("%");

      lcd.setCursor(0,1);

      lcd.print("Risk:");

      lcd.print(riskScore);

      lcd.print("%");

      break;

    case 3:

      lcd.setCursor(0,0);
      lcd.print("Imbalance");

      lcd.setCursor(0,1);

      lcd.print(imbalance);

      lcd.print("mV");

      break;

    case 4:

      lcd.setCursor(0,0);
      lcd.print("Recommendation");

      lcd.setCursor(0,1);

      if(recommendation.length()>16)
        lcd.print(recommendation.substring(0,16));
      else
        lcd.print(recommendation);

      break;
  }

  lcdPage++;

  if(lcdPage > 4)
    lcdPage = 0;
}

//----------------------------------------------------
// Main Enterprise Task
//----------------------------------------------------

void enterpriseTask()
{
  readSensors();

  updateTrend();

  calculateEnterpriseRisk();

  updateStateMachine();

  updateOutputs();

  generateRecommendation();

  monitorStateChange();

  updateDashboard();

  rotateLCD();

  sendAlerts();
}
