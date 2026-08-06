#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define NUM_CELLS 4

const uint8_t cellPins[NUM_CELLS] = {34, 35, 32, 33};

const int GREEN_LED = 2;
const int YELLOW_LED = 4;
const int RED_LED = 5;
const int BUZZER = 18;

LiquidCrystal_I2C lcd(0x27, 16, 2);

struct BMSData {
  float cellVoltage[NUM_CELLS];
  int weakest;
  int strongest;
  float imbalance;
  float previousImbalance;
  bool increasing;
  float soc;
  float threshold;
};

BMSData bms;

float adcToVoltage(int raw) {
  return (raw / 4095.0) * 4.2;   // Simulated cell voltage: 0–4.2V
}

float estimateSOC(float avgVoltage) {
  if (avgVoltage >= 4.20) return 100;
  if (avgVoltage <= 3.00) return 0;
  return ((avgVoltage - 3.0) / 1.2) * 100.0;
}

float adaptiveThreshold(float soc) {
  if (soc > 80) return 0.05;
  if (soc > 40) return 0.08;
  return 0.12;
}

void analyzeBattery(BMSData &d) {

  float minV = 100;
  float maxV = 0;
  float sum = 0;

  d.weakest = 0;
  d.strongest = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    int raw = analogRead(cellPins[i]);
    d.cellVoltage[i] = adcToVoltage(raw);
    sum += d.cellVoltage[i];

    if (d.cellVoltage[i] < minV) {
      minV = d.cellVoltage[i];
      d.weakest = i;
    }

    if (d.cellVoltage[i] > maxV) {
      maxV = d.cellVoltage[i];
      d.strongest = i;
    }
  }

  d.previousImbalance = d.imbalance;
  d.imbalance = maxV - minV;
  d.increasing = (d.imbalance > d.previousImbalance);

  float avg = sum / NUM_CELLS;
  d.soc = estimateSOC(avg);
  d.threshold = adaptiveThreshold(d.soc);
}

void updateOutputs(const BMSData &d) {

  noTone(BUZZER);

  if (d.imbalance < d.threshold * 0.6) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
  }
  else if (d.imbalance < d.threshold) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
  else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    tone(BUZZER, 2000);
  }
}

void displayLCD(const BMSData &d) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Imb:");
  lcd.print(d.imbalance,2);
  lcd.print(" T:");
  lcd.print(d.threshold,2);

  lcd.setCursor(0,1);
  lcd.print("W");
  lcd.print(d.weakest+1);
  lcd.print(" S");
  lcd.print(d.strongest+1);
  lcd.print(" ");
  lcd.print(d.increasing ? "UP":"DN");
}

void printSerial(const BMSData &d) {
  Serial.println("-----------");
  for(int i=0;i<NUM_CELLS;i++){
    Serial.print("Cell ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(d.cellVoltage[i],3);
    Serial.println(" V");
  }

  Serial.print("Weakest Cell : ");
  Serial.println(d.weakest+1);

  Serial.print("Strongest Cell : ");
  Serial.println(d.strongest+1);

  Serial.print("Imbalance : ");
  Serial.print(d.imbalance,3);
  Serial.println(" V");

  Serial.print("Trend : ");
  Serial.println(d.increasing ? "Increasing":"Decreasing/Stable");

  Serial.print("Estimated SoC : ");
  Serial.print(d.soc,1);
  Serial.println("%");

  Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  lcd.init();
  lcd.backlight();
}

void loop() {

  analyzeBattery(bms);

  updateOutputs(bms);

  displayLCD(bms);

  printSerial(bms);

  delay(1000);
}
