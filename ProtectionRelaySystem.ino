/*
=========================================================
 Non-Blocking Protection Relay & Safety System
 Author : Shreyash Kulkarni
 Board  : ESP32 DevKit V1
=========================================================
 Features
 ✓ Fully Non-Blocking
 ✓ Professional State Machine
 ✓ Relay Hysteresis
 ✓ Sliding Window Filter
 ✓ Frozen Sensor Detection
 ✓ Jump Detection
 ✓ Out of Range Detection
 ✓ Timed Recovery
 ✓ JSON Logging
 ✓ LCD Page Rotation
=========================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

//==================== PIN DEFINITIONS ====================

#define POT1        34
#define POT2        35
#define POT3        32

#define BUTTON_FAULT 13
#define BUTTON_CLEAR 23

#define RELAY_PIN   19
#define BUZZER_PIN  18

#define RED_LED     2
#define GREEN_LED   4
#define YELLOW_LED  5

//==================== ADC SETTINGS =======================

const float ADC_REF = 3.30;
const int ADC_MAX = 4095;

//==================== SENSOR LIMITS ======================

const float MIN_VOLTAGE = 0.30;
const float MAX_VOLTAGE = 3.20;

// Relay Hysteresis

const float RELAY_ON_LEVEL  = 2.80;
const float RELAY_OFF_LEVEL = 2.60;

//==================== TIMERS =============================

const unsigned long SAMPLE_INTERVAL = 100;
const unsigned long LCD_INTERVAL = 2000;
const unsigned long RECOVERY_DELAY = 5000;
const unsigned long RELAY_DELAY = 1000;
const unsigned long FREEZE_TIME = 4000;
const unsigned long DEBOUNCE_TIME = 40;

//==================== FILTER =============================

const byte WINDOW = 5;

//==================== STATES =============================

enum SystemState
{
    NORMAL,
    DEGRADED,
    FAILSAFE,
    RECOVERY
};

SystemState currentState = NORMAL;
SystemState previousState = NORMAL;

//==================== FAULTS =============================

enum FaultType
{
    NO_FAULT,
    FROZEN_SENSOR,
    SENSOR_JUMP,
    OUT_OF_RANGE,
    MULTIPLE_FAULTS
};

FaultType currentFault = NO_FAULT;

//==================== RAW VALUES =========================

int raw1=0;
int raw2=0;
int raw3=0;

int prevRaw1=0;
int prevRaw2=0;
int prevRaw3=0;

float voltage1=0;
float voltage2=0;
float voltage3=0;

//==================== FILTER BUFFERS =====================

int filter1[WINDOW];
int filter2[WINDOW];
int filter3[WINDOW];

byte filterIndex=0;

float avg1=0;
float avg2=0;
float avg3=0;

//==================== FLAGS ==============================

bool relayState=false;

bool frozenFault=false;
bool jumpFault=false;
bool rangeFault=false;

//==================== TIMERS =============================

unsigned long sampleTimer=0;
unsigned long lcdTimer=0;
unsigned long relayTimer=0;
unsigned long recoveryTimer=0;
unsigned long freezeTimer=0;

unsigned long button1Timer=0;
unsigned long button2Timer=0;

//==================== BUTTONS ============================

bool lastButton1=HIGH;
bool lastButton2=HIGH;

bool button1State=HIGH;
bool button2State=HIGH;

//==================== LCD ================================

byte lcdPage=0;

//==================== FAULT INJECTION ====================

byte injectedFault=0;

//==================== FUNCTION PROTOTYPES ================

void readSensors();

void filterSensors();

void detectFrozenSensor();

void detectJump();

void detectRange();

void processSensors();

void updateFaults();

void updateRelay();

void updateStateMachine();

void updateLCD();

void updateLEDs();

void updateBuzzer();

void debounceButtons();

void checkButtons();

void logTransition(
const char* oldState,
const char* newState,
const char* fault);

//==================== SETUP ==============================

void setup()
{
    Serial.begin(115200);

    pinMode(POT1,INPUT);
    pinMode(POT2,INPUT);
    pinMode(POT3,INPUT);

    pinMode(BUTTON_FAULT,INPUT_PULLUP);
    pinMode(BUTTON_CLEAR,INPUT_PULLUP);

    pinMode(RELAY_PIN,OUTPUT);
    pinMode(BUZZER_PIN,OUTPUT);

    pinMode(RED_LED,OUTPUT);
    pinMode(GREEN_LED,OUTPUT);
    pinMode(YELLOW_LED,OUTPUT);

    digitalWrite(RELAY_PIN,LOW);
    digitalWrite(BUZZER_PIN,LOW);

    digitalWrite(RED_LED,LOW);
    digitalWrite(YELLOW_LED,LOW);
    digitalWrite(GREEN_LED,HIGH);

    lcd.init();
    lcd.backlight();

    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("Protection");

    lcd.setCursor(0,1);
    lcd.print("System Ready");

    for(int i=0;i<WINDOW;i++)
    {
        filter1[i]=0;
        filter2[i]=0;
        filter3[i]=0;
    }

    Serial.println("------------------------------------");
    Serial.println("Protection Relay Started");
    Serial.println("------------------------------------");
}
//======================================================
// PART 2
// Sensor Reading, Sliding Window Filter & Fault Detection
//======================================================

//--------------- Read ADC -----------------------------

void readSensors()
{
    raw1 = analogRead(POT1);
    raw2 = analogRead(POT2);
    raw3 = analogRead(POT3);

    voltage1 = (raw1 * ADC_REF) / ADC_MAX;
    voltage2 = (raw2 * ADC_REF) / ADC_MAX;
    voltage3 = (raw3 * ADC_REF) / ADC_MAX;
}

//--------------- Moving Average -----------------------

void filterSensors()
{
    filter1[filterIndex] = raw1;
    filter2[filterIndex] = raw2;
    filter3[filterIndex] = raw3;

    long sum1 = 0;
    long sum2 = 0;
    long sum3 = 0;

    for(int i = 0; i < WINDOW; i++)
    {
        sum1 += filter1[i];
        sum2 += filter2[i];
        sum3 += filter3[i];
    }

    avg1 = sum1 / (float)WINDOW;
    avg2 = sum2 / (float)WINDOW;
    avg3 = sum3 / (float)WINDOW;

    filterIndex++;

    if(filterIndex >= WINDOW)
        filterIndex = 0;
}

//--------------- Frozen Sensor ------------------------
// Uses tolerance of ±2 ADC counts instead of exact match

void detectFrozenSensor()
{
    static bool timerStarted = false;

    bool same =
        abs(raw1 - prevRaw1) <= 2 &&
        abs(raw2 - prevRaw2) <= 2 &&
        abs(raw3 - prevRaw3) <= 2;

    if(same)
    {
        if(!timerStarted)
        {
            freezeTimer = millis();
            timerStarted = true;
        }

        if(millis() - freezeTimer >= FREEZE_TIME)
            frozenFault = true;
    }
    else
    {
        timerStarted = false;
        frozenFault = false;
    }
}

//--------------- Jump Detection -----------------------

void detectJump()
{
    jumpFault = false;

    if(abs(raw1 - avg1) > 450)
        jumpFault = true;

    if(abs(raw2 - avg2) > 450)
        jumpFault = true;

    if(abs(raw3 - avg3) > 450)
        jumpFault = true;
}

//--------------- Out Of Range -------------------------

void detectRange()
{
    rangeFault = false;

    if(voltage1 < MIN_VOLTAGE || voltage1 > MAX_VOLTAGE)
        rangeFault = true;

    if(voltage2 < MIN_VOLTAGE || voltage2 > MAX_VOLTAGE)
        rangeFault = true;

    if(voltage3 < MIN_VOLTAGE || voltage3 > MAX_VOLTAGE)
        rangeFault = true;
}

//--------------- Process All Sensors ------------------

void processSensors()
{
    readSensors();

    filterSensors();

    detectFrozenSensor();

    detectJump();

    detectRange();

    prevRaw1 = raw1;
    prevRaw2 = raw2;
    prevRaw3 = raw3;
}

//--------------- Update Fault -------------------------

void updateFaults()
{
    currentFault = NO_FAULT;

    if(frozenFault)
        currentFault = FROZEN_SENSOR;

    if(jumpFault)
        currentFault = SENSOR_JUMP;

    if(rangeFault)
        currentFault = OUT_OF_RANGE;

    // Fault injection using Button 1
    switch(injectedFault)
    {
        case 1:
            currentFault = FROZEN_SENSOR;
            break;

        case 2:
            currentFault = SENSOR_JUMP;
            break;

        case 3:
            currentFault = OUT_OF_RANGE;
            break;

        case 4:
            currentFault = MULTIPLE_FAULTS;
            break;
    }
}

//--------------- Button Debounce ----------------------

void debounceButtons()
{
    bool reading1 = digitalRead(BUTTON_FAULT);
    bool reading2 = digitalRead(BUTTON_CLEAR);

    if(reading1 != lastButton1)
        button1Timer = millis();

    if(reading2 != lastButton2)
        button2Timer = millis();

    if(millis() - button1Timer > DEBOUNCE_TIME)
        button1State = reading1;

    if(millis() - button2Timer > DEBOUNCE_TIME)
        button2State = reading2;

    lastButton1 = reading1;
    lastButton2 = reading2;
}

//--------------- Button Functions ---------------------

void checkButtons()
{
    static bool lastFaultPress = HIGH;
    static bool lastClearPress = HIGH;

    // Button 1 cycles through fault types
    if(lastFaultPress == HIGH && button1State == LOW)
    {
        injectedFault++;

        if(injectedFault > 4)
            injectedFault = 0;

        Serial.print("Injected Fault = ");
        Serial.println(injectedFault);
    }

    // Button 2 clears all faults
    if(lastClearPress == HIGH && button2State == LOW)
    {
        injectedFault = 0;

        frozenFault = false;
        jumpFault = false;
        rangeFault = false;

        currentFault = NO_FAULT;

        Serial.println("Fault Cleared");
    }

    lastFaultPress = button1State;
    lastClearPress = button2State;
}
//======================================================
// PART 3
// Professional State Machine
// Relay Control
// Recovery Logic
// LEDs
// Buzzer
// JSON Logging
//======================================================

//---------------- State Names ------------------------

const char *stateName[] =
{
  "NORMAL",
  "DEGRADED",
  "FAILSAFE",
  "RECOVERY"
};

const char *faultName[] =
{
  "NONE",
  "FROZEN",
  "JUMP",
  "OUT_RANGE",
  "MULTIPLE"
};

//---------------- JSON Logger ------------------------

void logTransition(
const char* oldState,
const char* newState,
const char* fault)
{
    Serial.print("{\"time\":");
    Serial.print(millis());

    Serial.print(",\"previous\":\"");
    Serial.print(oldState);

    Serial.print("\",\"current\":\"");
    Serial.print(newState);

    Serial.print("\",\"fault\":\"");
    Serial.print(fault);

    Serial.println("\"}");
}

//---------------- Change State -----------------------

void changeState(SystemState newState)
{
    if(currentState == newState)
        return;

    previousState = currentState;

    logTransition(
        stateName[previousState],
        stateName[newState],
        faultName[currentFault]);

    currentState = newState;

    if(currentState == RECOVERY)
        recoveryTimer = millis();
}

//---------------- Relay Hysteresis -------------------

void updateRelay()
{
    float average =
    (voltage1+voltage2+voltage3)/3.0;

    // Anti chatter
    if(millis()-relayTimer < RELAY_DELAY)
        return;

    switch(currentState)
    {

        case NORMAL:

            if(!relayState &&
               average > RELAY_ON_LEVEL)
            {
                relayState = true;
                relayTimer = millis();
            }

            if(relayState &&
               average < RELAY_OFF_LEVEL)
            {
                relayState = false;
                relayTimer = millis();
            }

        break;

        case DEGRADED:

            // Keep relay ON

        break;

        case FAILSAFE:

            relayState = false;

        break;

        case RECOVERY:

            relayState = false;

        break;

    }

    digitalWrite(RELAY_PIN,relayState);
}

//---------------- LEDs ------------------------------

void updateLEDs()
{

    switch(currentState)
    {

        case NORMAL:

            digitalWrite(GREEN_LED,HIGH);
            digitalWrite(YELLOW_LED,LOW);
            digitalWrite(RED_LED,LOW);

        break;

        case DEGRADED:

            digitalWrite(GREEN_LED,LOW);

            digitalWrite(
            YELLOW_LED,
            millis()/500%2);

            digitalWrite(RED_LED,LOW);

        break;

        case FAILSAFE:

            digitalWrite(GREEN_LED,LOW);
            digitalWrite(YELLOW_LED,LOW);

            digitalWrite(
            RED_LED,
            millis()/250%2);

        break;

        case RECOVERY:

            digitalWrite(GREEN_LED,LOW);

            digitalWrite(
            YELLOW_LED,
            millis()/150%2);

            digitalWrite(RED_LED,LOW);

        break;

    }

}

//---------------- Buzzer ----------------------------

void updateBuzzer()
{

    static bool firstEntry=true;

    if(currentState!=FAILSAFE)
    {
        digitalWrite(BUZZER_PIN,LOW);
        firstEntry=true;
        return;
    }

    // Only beep for first 3 seconds

    if(firstEntry)
    {
        relayTimer=millis();
        firstEntry=false;
    }

    if(millis()-relayTimer<3000)
    {
        digitalWrite(
        BUZZER_PIN,
        millis()/200%2);
    }
    else
    {
        digitalWrite(BUZZER_PIN,LOW);
    }

}

//---------------- FSM -------------------------------

void updateStateMachine()
{

    switch(currentState)
    {

        //---------------- NORMAL ----------------

        case NORMAL:

            if(currentFault!=NO_FAULT)
            {
                changeState(DEGRADED);
            }

        break;

        //---------------- DEGRADED -------------

        case DEGRADED:

            if(currentFault==MULTIPLE_FAULTS)
            {
                changeState(FAILSAFE);
            }

            else if(currentFault==NO_FAULT)
            {
                changeState(RECOVERY);
            }

        break;

        //---------------- FAILSAFE -------------

        case FAILSAFE:

            if(currentFault==NO_FAULT)
            {
                changeState(RECOVERY);
            }

        break;

        //---------------- RECOVERY -------------

        case RECOVERY:

            if(currentFault!=NO_FAULT)
            {
                changeState(FAILSAFE);
            }

            else
            {
                if(millis()-recoveryTimer>=RECOVERY_DELAY)
                {
                    changeState(NORMAL);
                }
            }

        break;

    }

    updateRelay();

    updateLEDs();

    updateBuzzer();

}
//======================================================
// PART 4
// LCD + Main Loop + Final Integration
//======================================================

//---------------- LCD -------------------------------

void updateLCD()
{
    if(millis()-lcdTimer<LCD_INTERVAL)
        return;

    lcdTimer=millis();

    lcd.clear();

    switch(lcdPage)
    {

        //---------------- PAGE 1 ----------------

        case 0:

            lcd.setCursor(0,0);
            lcd.print("V1:");
            lcd.print(voltage1,2);

            lcd.setCursor(9,0);
            lcd.print("V2:");
            lcd.print(voltage2,2);

            lcd.setCursor(0,1);
            lcd.print("V3:");
            lcd.print(voltage3,2);

        break;

        //---------------- PAGE 2 ----------------

        case 1:

            lcd.setCursor(0,0);
            lcd.print("STATE");

            lcd.setCursor(0,1);
            lcd.print(stateName[currentState]);

        break;

        //---------------- PAGE 3 ----------------

        case 2:

            lcd.setCursor(0,0);
            lcd.print("FAULT");

            lcd.setCursor(0,1);
            lcd.print(faultName[currentFault]);

        break;

        //---------------- PAGE 4 ----------------

        case 3:

            lcd.setCursor(0,0);

            lcd.print("Relay:");

            if(relayState)
                lcd.print("ON ");
            else
                lcd.print("OFF");

            lcd.setCursor(0,1);

            if(currentState==RECOVERY)
            {
                lcd.print("Rec:");

                unsigned long remain =
                (RECOVERY_DELAY-
                (millis()-recoveryTimer))/1000;

                lcd.print(remain);
                lcd.print("s");
            }
            else
            {
                lcd.print("Ready");
            }

        break;

    }

    lcdPage++;

    if(lcdPage>3)
        lcdPage=0;
}

//---------------- MAIN LOOP ------------------------

void loop()
{

    // Non-blocking scheduler

    if(millis()-sampleTimer>=SAMPLE_INTERVAL)
    {

        sampleTimer=millis();

        debounceButtons();

        checkButtons();

        processSensors();

        updateFaults();

        updateStateMachine();

    }

    updateLCD();

}