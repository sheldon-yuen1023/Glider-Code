#include "Orientation.h"
#include "CANHandler.h"
#include "Telemetry.h"
#include <HardwareSerial.h>
#include "Pins.h"

HardwareSerial RS485(2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting system...");

  //initOrientation();            // Start IMU on Core 0
  initCAN();                    // Init CAN
  startCANReceiveTask();        // Start CAN read task

  RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX); // Init RS485
  Serial.println("RS485 initialized.");

  startTelemetryTask();         // Start telemetry task on Core 1
}

void loop() {
  /*
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
  */
  // Add other non-blocking tasks here if needed
}
