


#define BLYNK_TEMPLATE_ID   "TMPL3GypB2ZSC"
#define BLYNK_TEMPLATE_NAME "bms project"
#define BLYNK_AUTH_TOKEN    "NPJwpVBfCprEgPifwlc7M7xxMycOpPEB"   

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";



#define NUM_CELLS 3
const int CELL_PINS[NUM_CELLS] = {34, 35, 32};   
#define TEMP_PIN   33                             

#define GREEN_LED   2     
#define YELLOW_LED  4
#define RED_LED     5     
#define LINK_LED    16    

#define RELAY_PIN     19
#define BUZZER_PIN    18
#define BUTTON_FAULT  13  
#define BUTTON_CLEAR  23  

LiquidCrystal_I2C lcd(0x27, 16, 2);


int ent_riskScore = 0;   


struct BMSData {
  float cellVoltage[NUM_CELLS];
  int   weakest;
  int   strongest;
  float imbalance;          
  float previousImbalance;
  bool  increasing;
  float soc;                
  float threshold;          
};

BMSData bms;



enum SystemState { NORMAL, DEGRADED, FAILSAFE, RECOVERY, SHUTDOWN };
enum FaultType { NO_FAULT, FROZEN_SENSOR, SENSOR_JUMP, OUT_OF_RANGE, CELL_IMBALANCE, MULTIPLE_FAULTS };
struct TelemetryEvent {
  float cellV[NUM_CELLS];
  uint8_t weakestIdx, strongestIdx;
  bool relayClosed, faultActive;
  int8_t rssi;
  unsigned long timestamp;
};


float bms_adcToVoltage(int raw) {
  
  return 3.0f + (raw / 4095.0f) * 1.2f;   
}

float bms_estimateSOC(float avgVoltage) {
  if (avgVoltage >= 4.20) return 100;
  if (avgVoltage <= 3.00) return 0;
  return ((avgVoltage - 3.0) / 1.2) * 100.0;
}

float bms_adaptiveThreshold(float soc) {
  if (soc > 80) return 0.05;
  if (soc > 40) return 0.08;
  return 0.12;
}

void bms_analyze(BMSData &d, float cellV[NUM_CELLS]) {
  float minV = 100, maxV = 0, sum = 0;
  d.weakest = 0;
  d.strongest = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    d.cellVoltage[i] = cellV[i];
    sum += d.cellVoltage[i];
    if (d.cellVoltage[i] < minV) { minV = d.cellVoltage[i]; d.weakest = i; }
    if (d.cellVoltage[i] > maxV) { maxV = d.cellVoltage[i]; d.strongest = i; }
  }

  d.previousImbalance = d.imbalance;
  d.imbalance = maxV - minV;
  d.increasing = (d.imbalance > d.previousImbalance);

  float avg = sum / NUM_CELLS;
  d.soc = bms_estimateSOC(avg);
  d.threshold = bms_adaptiveThreshold(d.soc);
}



SystemState currentState = NORMAL;
SystemState previousState = NORMAL;

FaultType currentFault = NO_FAULT;

const char *stateName[] = { "NORMAL", "DEGRADED", "FAILSAFE", "RECOVERY", "SHUTDOWN" };
const char *faultName[] = { "NONE", "FROZEN", "JUMP", "OUT_RANGE", "IMBALANCE", "MULTIPLE" };

const float ADC_REF = 3.30;
const int   ADC_MAX = 4095;
const float MIN_VOLTAGE = 0.30;
const float MAX_VOLTAGE = 3.20;
const float RELAY_ON_LEVEL  = 2.80;
const float RELAY_OFF_LEVEL = 2.60;

const unsigned long RECOVERY_DELAY = 5000;
const unsigned long RELAY_DELAY    = 1000;
const unsigned long FREEZE_TIME    = 4000;
const unsigned long DEBOUNCE_TIME  = 40;
const byte WINDOW = 5;

int raw[NUM_CELLS] = {0};
int prevRaw[NUM_CELLS] = {0};
float relayVoltage[NUM_CELLS] = {0};   

int filterBuf[NUM_CELLS][WINDOW];
byte filterIndex = 0;
float filterAvg[NUM_CELLS] = {0};

bool relayState = false;
bool frozenFault = false, jumpFault = false, rangeFault = false;
byte injectedFault = 0;   

unsigned long relayTimer = 0, recoveryTimer = 0, freezeTimer = 0;
unsigned long button1Timer = 0, button2Timer = 0;
bool lastButton1 = HIGH, lastButton2 = HIGH;
bool button1State = HIGH, button2State = HIGH;
bool lastFaultPress = HIGH, lastClearPress = HIGH;

void fsm_logTransition(const char* oldS, const char* newS, const char* fault) {
  Serial.print("{\"time\":"); Serial.print(millis());
  Serial.print(",\"previous\":\""); Serial.print(oldS);
  Serial.print("\",\"current\":\""); Serial.print(newS);
  Serial.print("\",\"fault\":\""); Serial.print(fault);
  Serial.println("\"}");
}

void fsm_changeState(SystemState s) {
  if (currentState == s) return;
  previousState = currentState;
  fsm_logTransition(stateName[previousState], stateName[s], faultName[currentFault]);
  currentState = s;
  if (currentState == RECOVERY) recoveryTimer = millis();
}

void relay_readSensors() {
  for (int i = 0; i < NUM_CELLS; i++) {
    raw[i] = analogRead(CELL_PINS[i]);
    relayVoltage[i] = (raw[i] * ADC_REF) / ADC_MAX;
  }
}

void relay_filterSensors() {
  for (int i = 0; i < NUM_CELLS; i++) filterBuf[i][filterIndex] = raw[i];

  for (int i = 0; i < NUM_CELLS; i++) {
    long sum = 0;
    for (int w = 0; w < WINDOW; w++) sum += filterBuf[i][w];
    filterAvg[i] = sum / (float)WINDOW;
  }

  filterIndex++;
  if (filterIndex >= WINDOW) filterIndex = 0;
}

void relay_detectFrozen() {
  static bool timerStarted = false;
  bool same = true;
  for (int i = 0; i < NUM_CELLS; i++)
    if (abs(raw[i] - prevRaw[i]) > 2) same = false;

  if (same) {
    if (!timerStarted) { freezeTimer = millis(); timerStarted = true; }
    if (millis() - freezeTimer >= FREEZE_TIME) frozenFault = true;
  } else {
    timerStarted = false;
    frozenFault = false;
  }
}

void relay_detectJump() {
  jumpFault = false;
  for (int i = 0; i < NUM_CELLS; i++)
    if (abs(raw[i] - filterAvg[i]) > 450) jumpFault = true;
}

void relay_detectRange() {
  rangeFault = false;
  for (int i = 0; i < NUM_CELLS; i++)
    if (relayVoltage[i] < MIN_VOLTAGE || relayVoltage[i] > MAX_VOLTAGE) rangeFault = true;
}

void relay_processSensors() {
  relay_readSensors();
  relay_filterSensors();
  relay_detectFrozen();
  relay_detectJump();
  relay_detectRange();
  for (int i = 0; i < NUM_CELLS; i++) prevRaw[i] = raw[i];
}

void relay_updateFaults() {
  currentFault = NO_FAULT;
  if (frozenFault) currentFault = FROZEN_SENSOR;
  if (jumpFault)   currentFault = SENSOR_JUMP;
  if (rangeFault)  currentFault = OUT_OF_RANGE;
  if (bms.imbalance > bms.threshold) currentFault = CELL_IMBALANCE;  

  switch (injectedFault) {          
    case 1: currentFault = FROZEN_SENSOR; break;
    case 2: currentFault = SENSOR_JUMP; break;
    case 3: currentFault = OUT_OF_RANGE; break;
    case 4: currentFault = MULTIPLE_FAULTS; break;
  }
}

void relay_debounceButtons() {
  bool r1 = digitalRead(BUTTON_FAULT);
  bool r2 = digitalRead(BUTTON_CLEAR);
  if (r1 != lastButton1) button1Timer = millis();
  if (r2 != lastButton2) button2Timer = millis();
  if (millis() - button1Timer > DEBOUNCE_TIME) button1State = r1;
  if (millis() - button2Timer > DEBOUNCE_TIME) button2State = r2;
  lastButton1 = r1;
  lastButton2 = r2;
}

void relay_checkButtons() {
  if (button1State == LOW && lastFaultPress == HIGH) {
    injectedFault = (injectedFault + 1) % 5;
    Serial.print("Injected fault set to: ");
    Serial.println(faultName[injectedFault == 0 ? NO_FAULT : injectedFault]);
  }

  if (button2State == LOW && lastClearPress == HIGH) {
    injectedFault = 0;
    frozenFault = jumpFault = rangeFault = false;
    currentFault = NO_FAULT;
    Serial.println("Fault Cleared");
    if (currentState == SHUTDOWN) fsm_changeState(RECOVERY);   
  }

  lastFaultPress = button1State;
  lastClearPress = button2State;
}

void relay_updateRelay() {
  float average = (relayVoltage[0] + relayVoltage[1] + relayVoltage[2]) / 3.0;
  if (millis() - relayTimer < RELAY_DELAY) return;

  switch (currentState) {
    case NORMAL:
      if (!relayState && average > RELAY_ON_LEVEL)  { relayState = true;  relayTimer = millis(); }
      if (relayState  && average < RELAY_OFF_LEVEL) { relayState = false; relayTimer = millis(); }
      break;
    case DEGRADED:  break;                 
    case FAILSAFE:  relayState = false; break;
    case RECOVERY:  relayState = false; break;
    case SHUTDOWN:  relayState = false; break;
  }
  digitalWrite(RELAY_PIN, relayState);
}

void relay_updateLEDs() {
  digitalWrite(GREEN_LED, currentState == NORMAL);
  digitalWrite(YELLOW_LED, currentState == DEGRADED || (currentState == RECOVERY && millis() / 150 % 2));
  digitalWrite(RED_LED, (currentState == FAILSAFE && millis() / 250 % 2) || currentState == SHUTDOWN);
}

void relay_updateBuzzer() {
  static bool firstEntry = true;
  if (currentState != FAILSAFE && currentState != SHUTDOWN) {
    digitalWrite(BUZZER_PIN, LOW);
    firstEntry = true;
    return;
  }
  if (currentState == SHUTDOWN) { digitalWrite(BUZZER_PIN, HIGH); return; }  

  if (firstEntry) { relayTimer = millis(); firstEntry = false; }
  digitalWrite(BUZZER_PIN, (millis() - relayTimer < 3000) ? (millis() / 200 % 2) : LOW);
}

void fsm_update() {
  switch (currentState) {
    case NORMAL:
      if (currentFault != NO_FAULT) fsm_changeState(DEGRADED);
      break;

    case DEGRADED:
      if (currentFault == MULTIPLE_FAULTS) fsm_changeState(FAILSAFE);
      else if (currentFault == NO_FAULT)   fsm_changeState(RECOVERY);
      break;

    case FAILSAFE:
      if (ent_riskScore >= 100) fsm_changeState(SHUTDOWN);          
      else if (currentFault == NO_FAULT) fsm_changeState(RECOVERY);
      break;

    case RECOVERY:
      if (currentFault != NO_FAULT) fsm_changeState(FAILSAFE);
      else if (millis() - recoveryTimer >= RECOVERY_DELAY) fsm_changeState(NORMAL);
      break;

    case SHUTDOWN:
      
      break;
  }

  relay_updateRelay();
  relay_updateLEDs();
  relay_updateBuzzer();
}



#define QUEUE_SIZE 20


class OfflineQueue {
  public:
    TelemetryEvent buf[QUEUE_SIZE];
    int head = 0, tail = 0, count = 0;
    bool isFull()  { return count >= QUEUE_SIZE; }
    bool isEmpty() { return count == 0; }
    int  depth()   { return count; }
    void push(const TelemetryEvent &ev) {
      buf[head] = ev;
      head = (head + 1) % QUEUE_SIZE;
      if (isFull()) tail = (tail + 1) % QUEUE_SIZE; else count++;
    }
    bool pop(TelemetryEvent &out) {
      if (isEmpty()) return false;
      out = buf[tail];
      tail = (tail + 1) % QUEUE_SIZE;
      count--;
      return true;
    }
};
OfflineQueue tel_offlineQueue;

enum ConnState { ST_WIFI_DISCONNECTED, ST_WIFI_CONNECTING, ST_WIFI_CONNECTED, ST_BLYNK_CONNECTING, ST_BLYNK_CONNECTED, ST_OUTAGE_SIMULATED };
ConnState tel_connState = ST_WIFI_DISCONNECTED;
unsigned long tel_stateEnteredAt = 0;
const unsigned long WIFI_CONNECT_TIMEOUT_MS  = 8000;
const unsigned long BLYNK_CONNECT_TIMEOUT_MS = 12000;
bool tel_simulateOutage = false;

float tel_lastSentCellV[NUM_CELLS];
bool  tel_lastSentRelay = true, tel_lastSentFault = false;
int8_t tel_lastRSSI = -100;
const float VOLTAGE_DELTA_THRESHOLD = 0.02f;
unsigned long tel_lastRSSISample = 0, tel_lastHeartbeat = 0;
const unsigned long HEARTBEAT_MS = 30000UL;
bool tel_wasConnectedLastLoop = false;
BlynkTimer blynkTimer;

bool tel_isFullyConnected() { return tel_connState == ST_BLYNK_CONNECTED; }

const char* tel_connStateLabel() {
  switch (tel_connState) {
    case ST_WIFI_DISCONNECTED:  return "WiFi: down";
    case ST_WIFI_CONNECTING:    return "WiFi: conn..";
    case ST_WIFI_CONNECTED:     return "Blynk: conn..";
    case ST_BLYNK_CONNECTING:   return "Blynk: conn..";
    case ST_BLYNK_CONNECTED:    return "Link: UP";
    case ST_OUTAGE_SIMULATED:   return "OUTAGE (demo)";
  }
  return "?";
}

void tel_runConnectionStateMachine() {
  unsigned long now = millis();
  switch (tel_connState) {
    case ST_WIFI_DISCONNECTED:
      if (tel_simulateOutage) return;
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, pass);
      tel_stateEnteredAt = now;
      tel_connState = ST_WIFI_CONNECTING;
      break;
    case ST_WIFI_CONNECTING:
      if (tel_simulateOutage) { WiFi.disconnect(true); tel_connState = ST_OUTAGE_SIMULATED; tel_stateEnteredAt = now; return; }
      if (WiFi.status() == WL_CONNECTED) { tel_connState = ST_WIFI_CONNECTED; tel_stateEnteredAt = now; }
      else if (now - tel_stateEnteredAt > WIFI_CONNECT_TIMEOUT_MS) { WiFi.disconnect(true); tel_connState = ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; }
      break;
    case ST_WIFI_CONNECTED:
      if (tel_simulateOutage || WiFi.status() != WL_CONNECTED) { tel_connState = tel_simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; return; }
      Blynk.config(BLYNK_AUTH_TOKEN);
      Blynk.connect(0);
      tel_connState = ST_BLYNK_CONNECTING;
      tel_stateEnteredAt = now;
      break;
    case ST_BLYNK_CONNECTING:
      if (tel_simulateOutage || WiFi.status() != WL_CONNECTED) { tel_connState = tel_simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; return; }
      if (Blynk.connected()) { tel_connState = ST_BLYNK_CONNECTED; tel_stateEnteredAt = now; }
      else if (now - tel_stateEnteredAt > BLYNK_CONNECT_TIMEOUT_MS) { tel_connState = ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; }
      break;
    case ST_BLYNK_CONNECTED:
      if (tel_simulateOutage || WiFi.status() != WL_CONNECTED || !Blynk.connected()) { tel_connState = tel_simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; }
      break;
    case ST_OUTAGE_SIMULATED:
      if (!tel_simulateOutage) { tel_connState = ST_WIFI_DISCONNECTED; tel_stateEnteredAt = now; }
      break;
  }
}

void tel_sendEventToBlynk(const TelemetryEvent &ev, bool wasQueued) {
  if (!tel_isFullyConnected()) return;
  Blynk.virtualWrite(V0, ev.cellV[0]);
  Blynk.virtualWrite(V1, ev.cellV[1]);
  Blynk.virtualWrite(V2, ev.cellV[2]);
  Blynk.virtualWrite(V3, ev.cellV[ev.weakestIdx]);
  Blynk.virtualWrite(V4, ev.cellV[ev.strongestIdx]);
  Blynk.virtualWrite(V5, bms.imbalance);
  Blynk.virtualWrite(V6, bms.soc);
  Blynk.virtualWrite(V7, ev.relayClosed ? 1 : 0);
  Blynk.virtualWrite(V8, ev.faultActive ? 1 : 0);
  Blynk.virtualWrite(V9, ev.rssi);
  Blynk.virtualWrite(V10, tel_offlineQueue.depth());
  int health = (ev.rssi <= -90) ? 0 : (ev.rssi <= -70 ? 1 : 2);
  Blynk.virtualWrite(V11, health);
  char line[100];
  snprintf(line, sizeof(line), "[%s] t=%lums relay=%s fault=%s state=%s",
           wasQueued ? "QUEUED" : "LIVE", ev.timestamp,
           ev.relayClosed ? "CLOSED" : "OPEN",
           ev.faultActive ? "FAULT" : "OK", stateName[currentState]);
  Blynk.virtualWrite(V12, line);
  Serial.println(line);
}

void tel_captureAndDispatch() {
  TelemetryEvent ev;
  for (int i = 0; i < NUM_CELLS; i++) ev.cellV[i] = bms.cellVoltage[i];
  ev.weakestIdx = bms.weakest;
  ev.strongestIdx = bms.strongest;
  ev.relayClosed = relayState;
  ev.faultActive = (currentFault != NO_FAULT);
  ev.rssi = tel_lastRSSI;
  ev.timestamp = millis();

  for (int i = 0; i < NUM_CELLS; i++) tel_lastSentCellV[i] = ev.cellV[i];
  tel_lastSentRelay = ev.relayClosed;
  tel_lastSentFault = ev.faultActive;

  if (tel_isFullyConnected()) tel_sendEventToBlynk(ev, false);
  else {
    tel_offlineQueue.push(ev);
    Serial.print("[Queue] stored offline. Depth = ");
    Serial.println(tel_offlineQueue.depth());
  }
}

void tel_drainOfflineQueue() {
  TelemetryEvent ev;
  while (tel_offlineQueue.pop(ev)) {
    tel_sendEventToBlynk(ev, true);
    Blynk.run();
  }
}

void tel_handleEventDetection() {
  bool changed = false;
  for (int i = 0; i < NUM_CELLS; i++)
    if (fabs(bms.cellVoltage[i] - tel_lastSentCellV[i]) >= VOLTAGE_DELTA_THRESHOLD) changed = true;
  if (relayState != tel_lastSentRelay) changed = true;
  if ((currentFault != NO_FAULT) != tel_lastSentFault) changed = true;
  if (changed) tel_captureAndDispatch();
}

void tel_sampleRSSI() {
  tel_lastRSSI = (WiFi.status() == WL_CONNECTED) ? (int8_t)WiFi.RSSI() : -100;
  digitalWrite(LINK_LED, tel_isFullyConnected());
}

BLYNK_WRITE(V13) { injectedFault = param.asInt() ? 1 : 0; }         
BLYNK_WRITE(V14) { tel_simulateOutage = param.asInt(); }             



int ent_batteryHealth = 100;
int ent_faultCount = 0;
unsigned long ent_bootTime = 0;
float ent_temperature = 0;

String ent_faultHistory[10];
int ent_faultIndex = 0;
String ent_recommendation = "";

const int TREND_SIZE = 20;
float ent_imbalanceHistory[TREND_SIZE];
int ent_trendIndex = 0;

void ent_updateTrend() {
  ent_imbalanceHistory[ent_trendIndex] = bms.imbalance * 1000;   
  ent_trendIndex = (ent_trendIndex + 1) % TREND_SIZE;
}

float ent_averageImbalance() {
  float sum = 0;
  for (int i = 0; i < TREND_SIZE; i++) sum += ent_imbalanceHistory[i];
  return sum / TREND_SIZE;
}

void ent_calculateRisk() {
  int imbalanceRisk = constrain((bms.imbalance * 1000) / 2, 0, 40);
  int tempRisk = (ent_temperature > 60) ? 30 : (ent_temperature > 45 ? 15 : 0);
  int socRisk  = (bms.soc < 20) ? 30 : (bms.soc < 40 ? 15 : 0);
  ent_riskScore = constrain(imbalanceRisk + tempRisk + socRisk, 0, 100);
  ent_batteryHealth = 100 - ent_riskScore;
}

void ent_logFault(String msg) {
  ent_faultHistory[ent_faultIndex] = msg;
  ent_faultIndex = (ent_faultIndex + 1) % 10;
  ent_faultCount++;
}

void ent_monitorStateChange() {
  static SystemState lastLogged = NORMAL;
  if (lastLogged != currentState) {
    ent_logFault("State Changed -> " + String(stateName[currentState]));
    lastLogged = currentState;
  }
}

void ent_generateRecommendation() {
  switch (currentState) {
    case NORMAL:   ent_recommendation = "Battery Healthy"; break;
    case DEGRADED: ent_recommendation = "Monitor Cell Imbalance"; break;
    case FAILSAFE: ent_recommendation = "Perform Battery Balancing"; break;
    case RECOVERY: ent_recommendation = "Verifying Recovery"; break;
    case SHUTDOWN: ent_recommendation = "Replace Faulty Cell"; break;
  }
  if (ent_temperature > 60) ent_recommendation += " | Cooling Required";
  if (bms.soc < 20) ent_recommendation += " | Recharge Battery";
}

String ent_getUptime() {
  unsigned long sec = (millis() - ent_bootTime) / 1000;
  int hr = sec / 3600; sec %= 3600;
  int mn = sec / 60; sec %= 60;
  return String(hr) + "h " + String(mn) + "m " + String(sec) + "s";
}

String ent_executiveSummary() {
  return "State:" + String(stateName[currentState]) +
         " | Health:" + String(ent_batteryHealth) + "%" +
         " | Risk:" + String(ent_riskScore) + "%" +
         " | Faults:" + String(ent_faultCount);
}

void ent_updateDashboard() {
  if (!tel_isFullyConnected()) return;
  Blynk.virtualWrite(V15, stateName[currentState]);
  Blynk.virtualWrite(V16, ent_riskScore);
  Blynk.virtualWrite(V17, ent_batteryHealth);
  Blynk.virtualWrite(V18, ent_faultCount);
  Blynk.virtualWrite(V19, ent_getUptime());
  Blynk.virtualWrite(V20, ent_recommendation);
  Blynk.virtualWrite(V21, ent_executiveSummary());
}

void ent_analyticsTask() {
  ent_temperature = map(analogRead(TEMP_PIN), 0, 4095, 20, 80);
  ent_updateTrend();
  ent_calculateRisk();
  ent_monitorStateChange();
  ent_generateRecommendation();
  ent_updateDashboard();
}

BLYNK_CONNECTED() {
  Serial.println("Blynk Connected");
  Blynk.syncAll();
}



String lcd_oldL0 = "", lcd_oldL1 = "";
unsigned long lcd_lastRefresh = 0;
const unsigned long LCD_REFRESH_INTERVAL = 200;      

unsigned long lcd_lastPage = 0;
const unsigned long LCD_PAGE_INTERVAL = 3000;
int lcd_page = 0;
const int LCD_PAGE_COUNT = 6;  

void lcd_drawLine(uint8_t row, String txt) {
  while (txt.length() < 16) txt += " ";
  txt = txt.substring(0, 16);
  String &old = (row == 0) ? lcd_oldL0 : lcd_oldL1;
  for (int i = 0; i < 16; i++) {
    if (old.length() < 16 || old[i] != txt[i]) {
      lcd.setCursor(i, row);
      lcd.print(txt[i]);
    }
  }
  old = txt;
}

void lcd_showFault() {
  lcd_drawLine(0, "*** FAULT ***");
  lcd_drawLine(1, String(faultName[currentFault]) + " " + stateName[currentState]);
}

void lcd_showPage(int page) {
  switch (page) {
    case 0:  // Task 1 — battery cells
      lcd_drawLine(0, "C1:" + String(bms.cellVoltage[0], 2) + " C2:" + String(bms.cellVoltage[1], 2));
      lcd_drawLine(1, "Imb:" + String(bms.imbalance, 2) + " SoC:" + String((int)bms.soc));
      break;
    case 1:  // Task 2/4 — state + relay
      lcd_drawLine(0, "State:" + String(stateName[currentState]));
      lcd_drawLine(1, "Relay:" + String(relayState ? "ON " : "OFF"));
      break;
    case 2:  // Task 5 — telemetry link
      lcd_drawLine(0, String(tel_connStateLabel()));
      lcd_drawLine(1, "RSSI:" + String(tel_lastRSSI) + " Q:" + String(tel_offlineQueue.depth()));
      break;
    case 3:  // Task 6 — risk / health
      lcd_drawLine(0, "Health:" + String(ent_batteryHealth) + "%");
      lcd_drawLine(1, "Risk:" + String(ent_riskScore) + "%");
      break;
    case 4:  // Task 6 — trend
      lcd_drawLine(0, "Imbalance Trend");
      lcd_drawLine(1, bms.increasing ? "UP" : "DOWN/STABLE");
      break;
    case 5:  // Task 6 — recommendation
      lcd_drawLine(0, "Recommend:");
      lcd_drawLine(1, ent_recommendation.length() > 16 ? ent_recommendation.substring(0, 16) : ent_recommendation);
      break;
  }
}

void lcd_update() {
  unsigned long now = millis();

  bool faultActive = (currentFault != NO_FAULT) || (currentState == FAILSAFE) || (currentState == SHUTDOWN);

  if (!faultActive && now - lcd_lastPage >= LCD_PAGE_INTERVAL) {
    lcd_page = (lcd_page + 1) % LCD_PAGE_COUNT;
    lcd_lastPage = now;
  }

  if (now - lcd_lastRefresh >= LCD_REFRESH_INTERVAL) {
    lcd_lastRefresh = now;
    if (faultActive) lcd_showFault();
    else lcd_showPage(lcd_page);
  }
}



unsigned long sampleTimer = 0;
const unsigned long SAMPLE_INTERVAL = 100;   

void setup() {
  Serial.begin(115200);
  ent_bootTime = millis();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(LINK_LED, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_FAULT, INPUT_PULLUP);
  pinMode(BUTTON_CLEAR, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GREEN_LED, HIGH);

  for (int c = 0; c < NUM_CELLS; c++)
    for (int w = 0; w < WINDOW; w++)
      filterBuf[c][w] = 0;

  for (int i = 0; i < NUM_CELLS; i++) tel_lastSentCellV[i] = -999;
  for (int i = 0; i < TREND_SIZE; i++) ent_imbalanceHistory[i] = 0;

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Integrated BMS");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
  delay(1000);   
  lcd.clear();

  tel_connState = ST_WIFI_DISCONNECTED;
  tel_stateEnteredAt = millis();

  Serial.println("=== Integrated BMS Project Boot Complete ===");
}

void loop() {
  unsigned long now = millis();

  tel_runConnectionStateMachine();
  if (tel_connState == ST_BLYNK_CONNECTING || tel_connState == ST_BLYNK_CONNECTED) Blynk.run();

 
  if (now - sampleTimer >= SAMPLE_INTERVAL) {
    sampleTimer = now;

    relay_debounceButtons();
    relay_checkButtons();
    relay_processSensors();                 

    float cellV[NUM_CELLS];
    for (int i = 0; i < NUM_CELLS; i++) cellV[i] = bms_adcToVoltage(raw[i]);
    bms_analyze(bms, cellV);                

    relay_updateFaults();
    fsm_update();                           
    tel_handleEventDetection();             
  }

  
  if (now - tel_lastRSSISample >= 2000) { tel_lastRSSISample = now; tel_sampleRSSI(); }

  
  if (now - tel_lastHeartbeat >= HEARTBEAT_MS) { tel_lastHeartbeat = now; tel_captureAndDispatch(); }

  
  bool connectedNow = tel_isFullyConnected();
  if (connectedNow && !tel_wasConnectedLastLoop) {
    Serial.println(">>> Connectivity restored - draining offline queue");
    tel_drainOfflineQueue();
    tel_captureAndDispatch();
  }
  tel_wasConnectedLastLoop = connectedNow;

  
  static unsigned long lastAnalytics = 0;
  if (now - lastAnalytics >= 2000) { lastAnalytics = now; ent_analyticsTask(); }

  
  lcd_update();
}
