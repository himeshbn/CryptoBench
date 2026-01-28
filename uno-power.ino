/**************************************************************************
 * UNO + INA219 Power Logger (Debounced Summary Mode)
 * --------------------------------------------------
 * - Waits for ESP32 trigger (GPIO4 → D2)
 * - Averages INA219 data while HIGH
 * - Prints exactly 1 summary per run (with debounce)
 **************************************************************************/

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#define TRIGGER_PIN 2
#define DEBOUNCE_DELAY 200  // ms to ignore false triggers after LOW

float voltageSum = 0, currentSum = 0, powerSum = 0;
unsigned long sampleCount = 0;
bool measuring = false;
unsigned long lastLowTime = 0;
int runNumber = 1;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  ina219.begin();
  pinMode(TRIGGER_PIN, INPUT);

  Serial.println("UNO + INA219 Power Logger (Debounced CSV Mode)");
  Serial.println("Run,Samples,AvgVoltage(V),AvgCurrent(mA),AvgPower(mW)");
}

void loop() {
  int triggerState = digitalRead(TRIGGER_PIN);

  if (triggerState == HIGH && !measuring) {
    // Start of a new ESP32 run
    measuring = true;
    voltageSum = 0; currentSum = 0; powerSum = 0; sampleCount = 0;
  }

  if (measuring && triggerState == HIGH) {
    // Actively measuring while trigger HIGH
    voltageSum += ina219.getBusVoltage_V();
    currentSum += ina219.getCurrent_mA();
    powerSum   += ina219.getPower_mW();
    sampleCount++;
    delay(5);
  }

  if (measuring && triggerState == LOW) {
    // End of run detected
    if (millis() - lastLowTime > DEBOUNCE_DELAY) {
      if (sampleCount > 0) {
        float avgV = voltageSum / sampleCount;
        float avgI = currentSum / sampleCount;
        float avgP = powerSum / sampleCount;

        Serial.print(runNumber); Serial.print(",");
        Serial.print(sampleCount); Serial.print(",");
        Serial.print(avgV, 3); Serial.print(",");
        Serial.print(avgI, 2); Serial.print(",");
        Serial.println(avgP, 2);

        runNumber++;
      }

      measuring = false;
      lastLowTime = millis();  // prevent double detection
      delay(100);              // small cooldown before next trigger
    }
  }
}
