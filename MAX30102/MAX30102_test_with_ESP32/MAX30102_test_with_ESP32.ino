/*
  ESP32 <--> MAX30102 (using SparkFun/particle MAX30105 library)

  Wiring / Circuit:
   - MAX30102 VIN  -> ESP32 3V3
   - MAX30102 GND  -> ESP32 GND
   - MAX30102 SDA  -> ESP32 GPIO21 (SDA)
   - MAX30102 SCL  -> ESP32 GPIO22 (SCL)
   - MAX30102 INT  -> (optional) leave disconnected or connect to any GPIO if you
                     want interrupts (not used in this sketch)

  Typical I2C address: 0x57 (handled by library)

  This sketch initializes the sensor, configures it and prints raw IR and RED
  samples from the FIFO to Serial.
*/

#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// Use these defines if you want to change default I2C pins for ESP32 
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialise I2C with explicit SDA, SCL pins (ESP32)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400kHz I2C

  Serial.println(F("Initializing MAX30102..."));

  // Try to initialize sensor. If you have a MAX30102 breakout, the MAX30105
  // library works for it as well.
  if(!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30102 not found. Check wiring/power."));
    while (1) {
      delay(1000);
    }
  }

  // You can tweak parameters inside setup() call:
  // Arguments: (ledMode, sampleAverage, ledPulseWidth, sampleRate, adcRange)
  // Good starting config:
  // ledMode: MAX30105_LED_MODE_RED_IR (read both Red and IR)
  // sampleAverage: 4
  // ledPulseWidth: 411 (us) => MAX30105_PULSE_WIDTH_411
  // sampleRate: 100 (Hz)
  // adcRange: 4096 (uA)
  particleSensor.setup(); // default configuration (works fine for raw reads)

  // Lower the pulse amplitude for the red/IR LEDs while testing:
  particleSensor.setPulseAmplitudeRed(0x0A);   // 0-255
  particleSensor.setPulseAmplitudeIR(0x0A);    // 0-255

  Serial.println(F("Place your finger (or reflect light) on the sensor."));
  delay(1000);
}

void loop() {
  // The library stores samples in an internal FIFO. We'll read available samples
  // and print the Red and IR values. We read as many samples as are available.
  // Note: For continuous streaming you may want to sample at fixed intervals.

  // Ensure new data available
  if (particleSensor.available()) {
    // read one sample at a time (you can loop to read more)
    uint32_t red = particleSensor.getRed(); // red LED reading
    uint32_t ir  = particleSensor.getIR();  // IR LED reading

    // Move to next sample in FIFO
    particleSensor.nextSample();

    // Print values
    Serial.print("RED: ");
    Serial.print(red);
    Serial.print("\tIR: ");
    Serial.println(ir);
  } else {
    // If nothing available, call check() to update library's internal state
    particleSensor.check(); // fetch a new sample into buffer if present
    // small delay to avoid flooding serial; adjust as needed
    delay(10);
  }

  // Optional: if you want to read a block of N samples:
  // for (int i=0; i<100; i++) { ... } and store them into arrays
}
