#include "Telemetry.h"
#include "CommandHandler.h"
#include <HardwareSerial.h>
#include "Orientation.h"
#include "CANHandler.h"
#include "Pins.h"

// Shared RS485 interface used across tasks (defined here, used in .cpp files)
HardwareSerial RS485(2);  // UART2 on ESP32 (ID = 2)

void setup() {
  RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX); // Init RS485
  Serial.println("RS485 initialized.");
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting system...");

  initOrientation();            // Start IMU on Core 0
  Serial.println("IMU initialized.");
  initCAN();                    // Init CAN
  Serial.println("CAN initialized.");
  startCANReceiveTask();        // Start CAN read task
  Serial.println("Reading CAN Now.");

  startTelemetryTask();         // Start telemetry task on Core 1
  Serial.println("Telemetry initialised");
  startCommandReceiveTask();    // Task that listens for RS485 commands and forwards them
  Serial.println("Command Receive Task initialised.");

  Serial.println("[SETUP] All tasks launched.");
}

void loop() {
  // Main loop is not used; all tasks run in parallel
}
