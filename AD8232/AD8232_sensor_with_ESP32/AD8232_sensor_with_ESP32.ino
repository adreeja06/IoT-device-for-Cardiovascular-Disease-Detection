/*
  AD8232 ECG with ESP32
  - Reads AD8232 OUT via ADC and detects beats (BPM)
  - SDA/SCL/I2C not used
  - Wiring (example):
      AD8232 VIN -> ESP32 3V3
      AD8232 GND -> ESP32 GND
      AD8232 OUT -> ESP32 GPIO4 (ADC)
      AD8232 LO+ -> ESP32 GPIO15 (digital input, optional)
      AD8232 LO- -> ESP32 GPIO2  (digital input, optional)
  - Safety: hobby/educational only. Not medical-grade.
*/

// ------------ user config ------------
const int PIN_ADC = 4;      // AD8232 OUT -> ADC pin (input-only recommended)
const int PIN_LO_PLUS = 15;  // AD8232 LO+ (optional lead-off detect)
const int PIN_LO_MINUS = 2;  // AD8232 LO- (optional lead-off detect)

const unsigned long SAMPLE_RATE = 250; // Hz (samples per second)
const unsigned long SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE; // microseconds
// -------------------------------------

// Filtering & detection state
float vFiltered = 0.0;
float lowpass = 0.0;
float highpass = 0.0;

const float alphaLP = 0.2;   // low-pass IIR coefficient (adjust)
const float alphaHP = 0.01;  // high-pass IIR coefficient for baseline removal (adjust)

// Adaptive threshold variables
float threshold = 0;
float baseline = 0;
const float baselineAlpha = 0.0005; // slow baseline update
const float threshOffset = 0.25;    // threshold above baseline (in normalized units)

// Peak detection
unsigned long lastBeatTime = 0;
const unsigned long minBeatInterval = 300; // ms (avoid false >200 BPM)
float bpm = 0;
float lastIBI = 0;
float bpmHistory[8];
int bpmIdx = 0;

// ADC conversion helpers
const int ADC_RES = 4095; // 12-bit on ESP32 (default)
const float VREF = 3.3;   // volts

// For serial timing
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 250; // ms

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LO_PLUS, INPUT);
  pinMode(PIN_LO_MINUS, INPUT);

  // Configure ADC width and attenuation for better dynamic range
  analogReadResolution(12); // 0..4095
  // Set attenuation for the ADC pin (ESP32-specific)
  // valid attns: ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(PIN_ADC, ADC_11db); // allows near full 0-3.3V range

  Serial.println("AD8232 -> ESP32 ECG reader");
  Serial.print("Sample rate: "); Serial.print(SAMPLE_RATE); Serial.println(" Hz");
  Serial.println("Ensure electrodes are attached: RA, LA, RL");
}

void loop() {
  static unsigned long nextMicros = 0;
  unsigned long now = micros();
  if (now < nextMicros) {
    // wait a bit
    delayMicroseconds(20);
    return;
  }
  nextMicros = now + SAMPLE_PERIOD_US;

  // 1) Read raw ADC
  int raw = analogRead(PIN_ADC);        // 0..4095
  float volts = (raw / float(ADC_RES)) * VREF; // convert to volts

  // Normalize signal roughly to 0..1
  // Many AD8232 outputs ~1.2-1.8V baseline; we will use filtering instead of strict normalization
  float signal = volts;

  // 2) Simple IIR low-pass filter
  lowpass = lowpass + alphaLP * (signal - lowpass);

  // 3) High-pass to remove baseline (subtract very slow baseline)
  baseline = baseline + baselineAlpha * (lowpass - baseline);
  highpass = lowpass - baseline;

  // 4) Smoothed filtered value (small IIR)
  vFiltered = vFiltered + alphaHP * (highpass - vFiltered);

  // 5) Lead-off detection (optional)
  bool leadOff = false;
  if (digitalRead(PIN_LO_PLUS) == HIGH || digitalRead(PIN_LO_MINUS) == HIGH) {
    leadOff = true;
  }

  // 6) Peak detection using rising-edge crossing above dynamic threshold
  // Update threshold slowly to track amplitude changes
  float absFilt = fabs(vFiltered);
  // Initialize threshold if it's zero
  if (threshold == 0) threshold = absFilt * 0.6 + 0.05;
  // slowly adapt threshold based on signal energy (keeps sensitivity)
  threshold = threshold * 0.999 + (absFilt * 0.001) * (1.0 + threshOffset);

  static bool wasAbove = false;
  bool isAbove = (vFiltered > threshold);

  unsigned long currentMillis = millis();
  if (!leadOff) {
    // rising edge detection
    if (!wasAbove && isAbove) {
      unsigned long t = currentMillis;
      if (lastBeatTime != 0) {
        unsigned long ibi = t - lastBeatTime; // ms
        if (ibi > minBeatInterval) {
          lastIBI = ibi;
          float instBpm = 60000.0f / float(ibi);
          // push to history for smoothing
          bpmHistory[bpmIdx % (int) (sizeof(bpmHistory)/sizeof(bpmHistory[0]))] = instBpm;
          bpmIdx++;
          // compute average BPM
          float sum = 0;
          int count = min(bpmIdx, (int)(sizeof(bpmHistory)/sizeof(bpmHistory[0])));
          for (int i=0;i<count;i++) sum += bpmHistory[i];
          bpm = sum / max(1, count);
        }
      }
      lastBeatTime = t;
    }
  } else {
    // lead off - reset detection to avoid false beats
    // optionally set bpm=0 or keep last value
  }
  wasAbove = isAbove;

  // 7) Serial output periodically (reduce spam)
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    Serial.print("RAW:");
    Serial.print(raw);
    Serial.print("\tV:");
    Serial.print(volts, 3);
    Serial.print("\tF:");
    Serial.print(vFiltered, 4);
    Serial.print("\tTH:");
    Serial.print(threshold, 4);
    Serial.print("\tLO:");
    Serial.print(leadOff ? "YES" : "NO");
    Serial.print("\tBPM:");
    if (bpm > 0.1) Serial.print(int(bpm + 0.5));
    else Serial.print("--");
    Serial.println();
  }

  // keep loop timing precise (we already used micros to schedule next sample)
}

// End of sketch
