
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

const float VOLTAGE_DELTA_THRESHOLD = 0.02f;
const float CELL_MIN_SAFE          = 3.00f;
const float CELL_MAX_SAFE          = 4.20f;
const float CELL_MIN_CLEAR         = 3.05f;
const float CELL_MAX_CLEAR         = 4.15f;
const unsigned long HEARTBEAT_MS   = 30000UL;

#define QUEUE_SIZE 20
#define NUM_CELLS 3

const int CELL_PINS[NUM_CELLS] = {32, 34, 35};
const int FAULT_BTN_PIN  = 13;
const int LED_HEALTH_PIN = 4;
const int LED_FAULT_PIN  = 2;
const int LED_LINK_PIN   = 5;
const int RELAY_PIN      = 19;
const int BUZZER_PIN     = 18;

const bool RELAY_ACTIVE_LOW = true;

const uint8_t LCD_I2C_ADDR = 0x27;
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

struct TelemetryEvent {
  float    cellV[NUM_CELLS];
  uint8_t  weakestIdx;
  uint8_t  strongestIdx;
  bool     relayClosed;
  bool     faultActive;
  int8_t   rssi;
  unsigned long timestamp;
};

class OfflineQueue {
  public:
    TelemetryEvent buf[QUEUE_SIZE];
    int head = 0;
    int tail = 0;
    int count = 0;

    bool isFull()  { return count >= QUEUE_SIZE; }
    bool isEmpty() { return count == 0; }
    int  depth()   { return count; }

    void push(const TelemetryEvent &ev) {
      buf[head] = ev;
      head = (head + 1) % QUEUE_SIZE;
      if (isFull()) {
        tail = (tail + 1) % QUEUE_SIZE;
      } else {
        count++;
      }
    }

    bool pop(TelemetryEvent &out) {
      if (isEmpty()) return false;
      out = buf[tail];
      tail = (tail + 1) % QUEUE_SIZE;
      count--;
      return true;
    }
};

OfflineQueue offlineQueue;

enum ConnState {
  ST_WIFI_DISCONNECTED,
  ST_WIFI_CONNECTING,
  ST_WIFI_CONNECTED,
  ST_BLYNK_CONNECTING,
  ST_BLYNK_CONNECTED,
  ST_OUTAGE_SIMULATED
};

ConnState connState = ST_WIFI_DISCONNECTED;
unsigned long stateEnteredAt = 0;
const unsigned long WIFI_CONNECT_TIMEOUT_MS  = 8000;
const unsigned long BLYNK_CONNECT_TIMEOUT_MS = 12000;

bool simulateOutage = false;

float cellV[NUM_CELLS];
float lastSentCellV[NUM_CELLS];
bool  relayClosed      = true;
bool  lastSentRelay    = true;
bool  faultActive      = false;
bool  lastSentFault    = false;
int8_t lastRSSI        = -100;

bool  faultInjectLatch     = false;
bool  wasConnectedLastLoop = false;

unsigned long lastSensorRead = 0;
unsigned long lastRSSISample = 0;
unsigned long lastHeartbeat  = 0;

int lastFaultBtnState = HIGH;
unsigned long lastFaultBtnEdge = 0;
const unsigned long DEBOUNCE_MS = 200;

BlynkTimer timer;

bool ledFaultOn = false;
unsigned long lastFaultBlinkToggle = 0;
const unsigned long FAULT_BLINK_MS = 300;

bool ledLinkOn = false;
unsigned long lastLinkBlinkToggle = 0;
const unsigned long LINK_BLINK_MS = 400;

enum LcdPage {
  PAGE_CELLS_12 = 0,
  PAGE_CELL_3,
  PAGE_STATUS,
  PAGE_LINK,
  LCD_PAGE_COUNT
};

LcdPage lcdPage = PAGE_CELLS_12;
unsigned long lastLcdPageSwitch = 0;
const unsigned long LCD_PAGE_MS = 3000;

char lcdLine0[LCD_COLS + 1] = "";
char lcdLine1[LCD_COLS + 1] = "";
char lcdLastLine0[LCD_COLS + 1] = "";
char lcdLastLine1[LCD_COLS + 1] = "";

void runConnectionStateMachine();
void readSensors();
void evaluateFaultAndRelay();
void checkSerialCommands();
void handleEventDetection();
void captureAndDispatchEvent(bool isLiveAttempt);
void sendEventToBlynk(const TelemetryEvent &ev, bool wasQueued);
void drainOfflineQueue();
void sampleRSSI();
void updateLocalIndicators();
bool isFullyConnected();
void updateLcd();
void lcdWriteLineIfChanged(uint8_t row, const char *text, char *lastBuf);
const char* connStateLabel();

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(FAULT_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_HEALTH_PIN, OUTPUT);
  pinMode(LED_FAULT_PIN, OUTPUT);
  pinMode(LED_LINK_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_HEALTH_PIN, HIGH);
  digitalWrite(LED_FAULT_PIN, LOW);
  digitalWrite(LED_LINK_PIN, LOW);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);

  for (int i = 0; i < NUM_CELLS; i++) {
    cellV[i] = 3.7f;
    lastSentCellV[i] = -999;
  }

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("BMS Telemetry"));
  lcd.setCursor(0, 1);
  lcd.print(F("Booting..."));
  strcpy(lcdLastLine0, "BMS Telemetry");
  strcpy(lcdLastLine1, "Booting...");

  connState = ST_WIFI_DISCONNECTED;
  stateEnteredAt = millis();
  lastLcdPageSwitch = millis();

  Serial.println(F("=== Event-Driven BMS Telemetry Boot ==="));
}

void loop() {
  unsigned long now = millis();

  runConnectionStateMachine();

  if (connState == ST_BLYNK_CONNECTING || connState == ST_BLYNK_CONNECTED) {
    Blynk.run();
  }

  checkSerialCommands();

  if (now - lastSensorRead >= 200) {
    lastSensorRead = now;
    readSensors();
    evaluateFaultAndRelay();
    updateLocalIndicators();
    handleEventDetection();
  }

  if (now - lastRSSISample >= 2000) {
    lastRSSISample = now;
    sampleRSSI();
  }

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    captureAndDispatchEvent(true);
  }

  bool connectedNow = (connState == ST_BLYNK_CONNECTED);
  if (connectedNow && !wasConnectedLastLoop) {
    Serial.println(F(">>> Connectivity restored - draining offline queue"));
    drainOfflineQueue();
    captureAndDispatchEvent(true);
  }
  wasConnectedLastLoop = connectedNow;

  updateLcd();
}

void runConnectionStateMachine() {
  unsigned long now = millis();

  switch (connState) {

    case ST_WIFI_DISCONNECTED:
      if (simulateOutage) return;
      Serial.println(F("[WiFi] Starting connection attempt..."));
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, pass);
      stateEnteredAt = now;
      connState = ST_WIFI_CONNECTING;
      break;

    case ST_WIFI_CONNECTING:
      if (simulateOutage) {
        WiFi.disconnect(true);
        connState = ST_OUTAGE_SIMULATED;
        stateEnteredAt = now;
        return;
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.print(F("[WiFi] Connected, IP: "));
        Serial.println(WiFi.localIP());
        connState = ST_WIFI_CONNECTED;
        stateEnteredAt = now;
      } else if (now - stateEnteredAt > WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println(F("[WiFi] Connect attempt timed out, backing off"));
        WiFi.disconnect(true);
        connState = ST_WIFI_DISCONNECTED;
        stateEnteredAt = now;
      }
      break;

    case ST_WIFI_CONNECTED:
      if (simulateOutage || WiFi.status() != WL_CONNECTED) {
        connState = simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED;
        stateEnteredAt = now;
        return;
      }
      Serial.println(F("[Blynk] Attempting Blynk connect..."));
      Blynk.config(BLYNK_AUTH_TOKEN);
      Blynk.connect(0);
      connState = ST_BLYNK_CONNECTING;
      stateEnteredAt = now;
      break;

    case ST_BLYNK_CONNECTING:
      if (simulateOutage || WiFi.status() != WL_CONNECTED) {
        connState = simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED;
        stateEnteredAt = now;
        return;
      }
      if (Blynk.connected()) {
        Serial.println(F("[Blynk] Connected."));
        connState = ST_BLYNK_CONNECTED;
        stateEnteredAt = now;
      } else if (now - stateEnteredAt > BLYNK_CONNECT_TIMEOUT_MS) {
        Serial.println(F("[Blynk] Connect attempt timed out, retrying"));
        connState = ST_WIFI_CONNECTED;
        stateEnteredAt = now;
      }
      break;

    case ST_BLYNK_CONNECTED:
      if (simulateOutage || WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
        Serial.println(F("[Link] Connection lost."));
        connState = simulateOutage ? ST_OUTAGE_SIMULATED : ST_WIFI_DISCONNECTED;
        stateEnteredAt = now;
      }
      break;

    case ST_OUTAGE_SIMULATED:
      if (!simulateOutage) {
        Serial.println(F("[WiFi] Outage cleared, resuming reconnection"));
        connState = ST_WIFI_DISCONNECTED;
        stateEnteredAt = now;
      }
      break;
  }
}

bool isFullyConnected() {
  return connState == ST_BLYNK_CONNECTED;
}

const char* connStateLabel() {
  switch (connState) {
    case ST_WIFI_DISCONNECTED:  return "WiFi: down";
    case ST_WIFI_CONNECTING:    return "WiFi: conn..";
    case ST_WIFI_CONNECTED:     return "Blynk: conn..";
    case ST_BLYNK_CONNECTING:   return "Blynk: conn..";
    case ST_BLYNK_CONNECTED:    return "Link: UP";
    case ST_OUTAGE_SIMULATED:   return "OUTAGE (demo)";
  }
  return "?";
}

void readSensors() {
  for (int i = 0; i < NUM_CELLS; i++) {
    int raw = analogRead(CELL_PINS[i]);
    float v = 3.0f + (raw / 4095.0f) * 1.2f;
    cellV[i] = v;
  }

  if (faultInjectLatch) {
    cellV[0] = 4.45f;
  }
}

void evaluateFaultAndRelay() {
  if (!faultActive) {
    bool triggered = false;
    for (int i = 0; i < NUM_CELLS; i++) {
      if (cellV[i] < CELL_MIN_SAFE || cellV[i] > CELL_MAX_SAFE) {
        triggered = true;
        break;
      }
    }
    faultActive = triggered;
  } else {
    bool allClear = true;
    for (int i = 0; i < NUM_CELLS; i++) {
      if (cellV[i] < CELL_MIN_CLEAR || cellV[i] > CELL_MAX_CLEAR) {
        allClear = false;
        break;
      }
    }
    faultActive = !allClear;
  }
  relayClosed = !faultActive;
}

void updateLocalIndicators() {
  unsigned long now = millis();

  digitalWrite(LED_HEALTH_PIN, faultActive ? LOW : HIGH);

  if (faultActive) {
    if (now - lastFaultBlinkToggle >= FAULT_BLINK_MS) {
      lastFaultBlinkToggle = now;
      ledFaultOn = !ledFaultOn;
      digitalWrite(LED_FAULT_PIN, ledFaultOn ? HIGH : LOW);
    }
  } else {
    ledFaultOn = false;
    digitalWrite(LED_FAULT_PIN, LOW);
  }

  digitalWrite(RELAY_PIN, relayClosed
                 ? (RELAY_ACTIVE_LOW ? LOW : HIGH)
                 : (RELAY_ACTIVE_LOW ? HIGH : LOW));

  digitalWrite(BUZZER_PIN, faultActive ? HIGH : LOW);

  if (isFullyConnected()) {
    ledLinkOn = true;
    digitalWrite(LED_LINK_PIN, HIGH);
  } else if (connState == ST_WIFI_DISCONNECTED || connState == ST_OUTAGE_SIMULATED) {
    ledLinkOn = false;
    digitalWrite(LED_LINK_PIN, LOW);
  } else {
    
    if (now - lastLinkBlinkToggle >= LINK_BLINK_MS) {
      lastLinkBlinkToggle = now;
      ledLinkOn = !ledLinkOn;
      digitalWrite(LED_LINK_PIN, ledLinkOn ? HIGH : LOW);
    }
  }
}

void checkSerialCommands() {
  unsigned long now = millis();

  int faultReading = digitalRead(FAULT_BTN_PIN);
  if (faultReading != lastFaultBtnState && (now - lastFaultBtnEdge) > DEBOUNCE_MS) {
    lastFaultBtnEdge = now;
    lastFaultBtnState = faultReading;
    if (faultReading == LOW) {
      faultInjectLatch = !faultInjectLatch;
      Serial.print(F("[Demo] Fault injection (button) "));
      Serial.println(faultInjectLatch ? F("ENABLED") : F("DISABLED"));
    }
  }

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'f' || c == 'F') {
      faultInjectLatch = !faultInjectLatch;
      Serial.print(F("[Demo] Fault injection (serial) "));
      Serial.println(faultInjectLatch ? F("ENABLED") : F("DISABLED"));
    } else if (c == 'o' || c == 'O') {
      simulateOutage = !simulateOutage;
      Serial.print(F("[Demo] Network outage simulation (serial) "));
      Serial.println(simulateOutage ? F("ENABLED") : F("DISABLED"));
    }
  }
}

BLYNK_WRITE(V12) {
  int v = param.asInt();
  faultInjectLatch = (v != 0);
  Serial.print(F("[Demo] Fault injection (Blynk) "));
  Serial.println(faultInjectLatch ? F("ENABLED") : F("DISABLED"));
}

BLYNK_WRITE(V13) {
  int v = param.asInt();
  simulateOutage = (v != 0);
  Serial.print(F("[Demo] Network outage simulation (Blynk) "));
  Serial.println(simulateOutage ? F("ENABLED") : F("DISABLED"));
}

void sampleRSSI() {
  if (WiFi.status() == WL_CONNECTED) {
    lastRSSI = (int8_t)WiFi.RSSI();
  } else {
    lastRSSI = -100;
  }
}

void handleEventDetection() {
  bool changed = false;

  for (int i = 0; i < NUM_CELLS; i++) {
    if (fabs(cellV[i] - lastSentCellV[i]) >= VOLTAGE_DELTA_THRESHOLD) {
      changed = true;
      break;
    }
  }
  if (relayClosed != lastSentRelay) changed = true;
  if (faultActive != lastSentFault) changed = true;

  if (changed) {
    captureAndDispatchEvent(true);
  }
}

void captureAndDispatchEvent(bool isLiveAttempt) {
  TelemetryEvent ev;
  for (int i = 0; i < NUM_CELLS; i++) ev.cellV[i] = cellV[i];

  uint8_t weakest = 0, strongest = 0;
  for (int i = 1; i < NUM_CELLS; i++) {
    if (cellV[i] < cellV[weakest])   weakest = i;
    if (cellV[i] > cellV[strongest]) strongest = i;
  }
  ev.weakestIdx   = weakest;
  ev.strongestIdx = strongest;
  ev.relayClosed  = relayClosed;
  ev.faultActive  = faultActive;
  ev.rssi         = lastRSSI;
  ev.timestamp    = millis();

  for (int i = 0; i < NUM_CELLS; i++) lastSentCellV[i] = cellV[i];
  lastSentRelay = relayClosed;
  lastSentFault = faultActive;

  if (isFullyConnected()) {
    sendEventToBlynk(ev, false);
  } else {
    offlineQueue.push(ev);
    Serial.print(F("[Queue] Event stored offline. Depth = "));
    Serial.println(offlineQueue.depth());
  }
}

void drainOfflineQueue() {
  TelemetryEvent ev;
  while (offlineQueue.pop(ev)) {
    sendEventToBlynk(ev, true);
    Blynk.run();
    delayMicroseconds(1);
  }
  Serial.println(F("[Queue] Drain complete."));
}

void sendEventToBlynk(const TelemetryEvent &ev, bool wasQueued) {
  if (!isFullyConnected()) return;

  Blynk.virtualWrite(V0, ev.cellV[0]);
  Blynk.virtualWrite(V1, ev.cellV[1]);
  Blynk.virtualWrite(V2, ev.cellV[2]);
  Blynk.virtualWrite(V4, ev.cellV[ev.weakestIdx]);
  Blynk.virtualWrite(V5, ev.cellV[ev.strongestIdx]);
  Blynk.virtualWrite(V6, ev.relayClosed ? 1 : 0);
  Blynk.virtualWrite(V7, ev.faultActive ? 1 : 0);
  Blynk.virtualWrite(V8, ev.rssi);
  Blynk.virtualWrite(V9, offlineQueue.depth());

  int health;
  if (ev.rssi <= -90) health = 0;
  else if (ev.rssi <= -70) health = 1;
  else health = 2;
  Blynk.virtualWrite(V10, health);

  char line[100];
  snprintf(line, sizeof(line),
           "[%s] t=%lums C1=%.2f C2=%.2f C3=%.2f relay=%s fault=%s",
           wasQueued ? "QUEUED" : "LIVE",
           ev.timestamp,
           ev.cellV[0], ev.cellV[1], ev.cellV[2],
           ev.relayClosed ? "CLOSED" : "OPEN",
           ev.faultActive ? "FAULT" : "OK");
  Blynk.virtualWrite(V11, line);
  Serial.println(line);
}

void lcdWriteLineIfChanged(uint8_t row, const char *text, char *lastBuf) {
  if (strcmp(text, lastBuf) != 0) {
    lcd.setCursor(0, row);
    char padded[LCD_COLS + 1];
    snprintf(padded, sizeof(padded), "%-16s", text);
    lcd.print(padded);
    strncpy(lastBuf, text, LCD_COLS);
    lastBuf[LCD_COLS] = '\0';
  }
}

void updateLcd() {
  unsigned long now = millis();

  if (faultActive) {
    lcdPage = PAGE_STATUS;
    lastLcdPageSwitch = now;
  } else if (now - lastLcdPageSwitch >= LCD_PAGE_MS) {
    lastLcdPageSwitch = now;
    lcdPage = (LcdPage)((lcdPage + 1) % LCD_PAGE_COUNT);
  }

  switch (lcdPage) {
    case PAGE_CELLS_12:
      snprintf(lcdLine0, sizeof(lcdLine0), "C1:%.2fV C2:%.2fV", cellV[0], cellV[1]);
      snprintf(lcdLine1, sizeof(lcdLine1), "Cells 1-2");
      break;

    case PAGE_CELL_3:
      snprintf(lcdLine0, sizeof(lcdLine0), "C3:%.2fV", cellV[2]);
      snprintf(lcdLine1, sizeof(lcdLine1), "Cell 3");
      break;

    case PAGE_STATUS:
      snprintf(lcdLine0, sizeof(lcdLine0), "Relay:%s", relayClosed ? "CLOSED" : "OPEN");
      snprintf(lcdLine1, sizeof(lcdLine1), "%s", faultActive ? "*** FAULT ***" : "Status: OK");
      break;

    case PAGE_LINK:
      snprintf(lcdLine0, sizeof(lcdLine0), "%s", connStateLabel());
      snprintf(lcdLine1, sizeof(lcdLine1), "RSSI:%ddBm Q:%d", lastRSSI, offlineQueue.depth());
      break;

    default:
      break;
  }

  lcdWriteLineIfChanged(0, lcdLine0, lcdLastLine0);
  lcdWriteLineIfChanged(1, lcdLine1, lcdLastLine1);
}
