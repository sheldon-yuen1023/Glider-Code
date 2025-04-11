#include "Orientation.h"

// Tracks the time of the last orientation update (in microseconds)
unsigned long lastOrientationUpdate = 0;

// Update interval in microseconds, based on the target filter rate (e.g. 200 Hz = 5000 µs)
const unsigned long updateInterval = 1000000 / FILTER_UPDATE_RATE_HZ;

// Controls how often orientation is printed to Serial
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 50000; // 50 ms = 20 Hz print rate

void setup() {
  Serial.begin(115200);  // Start serial output for logging
  while (!Serial);       // Wait for USB serial to connect (important on some boards)
  initOrientation();     // Initialize sensors and orientation filter
}

void loop() {
  unsigned long now = micros();

  // Check if it's time to update the orientation filter
  if (now - lastOrientationUpdate >= updateInterval) {
    lastOrientationUpdate = now;

    float pitch, roll, yaw;
    getOrientation(pitch, roll, yaw); // Read new orientation values

    // Check if it's time to print the current orientation to Serial
    if (now - lastPrintTime >= printInterval) {
      lastPrintTime = now;
      Serial.printf("Pitch: %.2f°, Roll: %.2f°, Yaw: %.2f°\n", pitch, roll, yaw);
    }
  }

  // Other tasks can be added here — non-blocking only
}

//hello
