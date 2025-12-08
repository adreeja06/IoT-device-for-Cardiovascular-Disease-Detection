#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// ---------------- MAX30102 CONFIG ----------------
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// ---------------- AD8232 CONFIG ----------------
const int PIN_ADC = 4;      
const int PIN_LO_PLUS = 15;
const int PIN_LO_MINUS = 2;

const unsigned long SAMPLE_RATE = 250; 
const unsigned long SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

// -------- ECG FILTER + BPM VARIABLES --------
float vFiltered = 0.0;
float lowpass = 0.0;
float highpass = 0.0;

const float alphaLP = 0.2;
const float alphaHP = 0.01;

float threshold = 0;
float baseline = 0;
const float baselineAlpha = 0.0005;
const float threshOffset = 0.25;

unsigned long lastBeatTime = 0;
const unsigned long minBeatInterval = 300;
float bpm = 0;
float bpmHistory[8];
int bpmIdx = 0;

const int ADC_RES = 4095;
const float VREF = 3.3;

// -------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  // ----- I2C INIT -----
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1) delay(1000);
  }

  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  // ----- ECG INIT -----
  pinMode(PIN_LO_PLUS, INPUT);
  pinMode(PIN_LO_MINUS, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC, ADC_11db);

  Serial.println("MAX30102 + AD8232 ECG Reader (ESP32)");
}

// -------------------------------------------------------------
//                       MAIN LOOP
// -------------------------------------------------------------
void loop() {

  // =============================================================
  //                1) READ ECG AT FIXED 250 Hz
  // =============================================================
  static unsigned long nextMicros = 0;
  unsigned long nowMicros = micros();

  if (nowMicros >= nextMicros) {
    nextMicros = nowMicros + SAMPLE_PERIOD_US;

    int raw = analogRead(PIN_ADC);
    float volts = (raw / float(ADC_RES)) * VREF;

    // Low pass filtering
    lowpass = lowpass + alphaLP * (volts - lowpass);

    // High pass (baseline removal)
    baseline = baseline + baselineAlpha * (lowpass - baseline);
    highpass = lowpass - baseline;

    // Final filtered output
    vFiltered = vFiltered + alphaHP * (highpass - vFiltered);

    // Lead-off
    bool leadOff = (digitalRead(PIN_LO_PLUS) == HIGH || digitalRead(PIN_LO_MINUS) == HIGH);

    // Adaptive threshold
    float absFilt = fabs(vFiltered);
    if (threshold == 0) threshold = absFilt * 0.6 + 0.05;
    threshold = threshold * 0.999 + (absFilt * 0.001) * (1 + threshOffset);

    // Peak detection
    static bool wasAbove = false;
    bool isAbove = (vFiltered > threshold);
    unsigned long t = millis();

    if (!leadOff) {
      if (!wasAbove && isAbove) {
        if (lastBeatTime != 0) {
          unsigned long ibi = t - lastBeatTime;
          if (ibi > minBeatInterval) {
            float instBPM = 60000.0 / ibi;
            bpmHistory[bpmIdx % 8] = instBPM;
            bpmIdx++;

            float sum = 0;
            int count = min(bpmIdx, 8);
            for (int i = 0; i < count; i++) sum += bpmHistory[i];
            bpm = sum / count;
          }
        }
        lastBeatTime = t;
      }
    }
    wasAbove = isAbove;

    // ECG Serial Output (no delay)
    Serial.print("ECG_RAW:"); Serial.print(raw);
    Serial.print("\tECG_FILT:"); Serial.print(vFiltered, 4);
    Serial.print("\tECG_BPM:");
    if (bpm > 1) Serial.print(bpm, 1); else Serial.print("--");
  }

  // =============================================================
  //          2) READ MAX30102 WHENEVER DATA AVAILABLE
  // =============================================================
  particleSensor.check();

  if (particleSensor.available()) {

    uint32_t red = particleSensor.getRed();
    uint32_t ir  = particleSensor.getIR();
    particleSensor.nextSample();

    // MAX30102 Serial Output
    Serial.print("\tRED:"); Serial.print(red);
    Serial.print("\tIR:"); Serial.print(ir);
  }

  Serial.println();   // complete one combined output line
}
