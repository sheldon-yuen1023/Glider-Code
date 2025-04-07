#include "Orientation.h"

unsigned long lastOrientationUpdate = 0;
const unsigned long updateInterval = 1000000 / FILTER_UPDATE_RATE_HZ;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000000; // Print every 1 second

// For measuring actual update frequency
unsigned long lastRateCheck = 0;
unsigned long updateCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  initOrientation();
}

void loop() {
  unsigned long now = micros();

  if (now - lastOrientationUpdate >= updateInterval) {
    lastOrientationUpdate = now;

    float pitch, roll, yaw;
    getOrientation(pitch, roll, yaw);

    // Count how many times we update
    updateCount++;

    // Print orientation once per second
    if (now - lastPrintTime >= printInterval) {
      lastPrintTime = now;
      Serial.printf("Pitch: %.2f°, Roll: %.2f°, Yaw: %.2f°\n", pitch, roll, yaw);
    }

    // Measure actual update frequency
    if (now - lastRateCheck >= 1000000) {  // every 1 second
      float actualRateHz = (float)updateCount / ((now - lastRateCheck) / 1000000.0f);
      Serial.printf("[Measured filter rate] %.1f Hz\n", actualRateHz);
      lastRateCheck = now;
      updateCount = 0;
    }
  }
}
