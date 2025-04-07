#include "Orientation.h"

unsigned long lastOrientationUpdate = 0;
const unsigned long updateInterval = 1000000 / FILTER_UPDATE_RATE_HZ;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 50000; // 1 Hz

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

    if (now - lastPrintTime >= printInterval) {
      lastPrintTime = now;
      Serial.printf("Pitch: %.2f°, Roll: %.2f°, Yaw: %.2f°\n", pitch, roll, yaw);
    }
  }
}
