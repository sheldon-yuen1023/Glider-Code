#include "CANHandler.h"
#include "Pins.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("VBD Node Initializing CAN...");

  initCAN();  // Initializes CAN and starts background send/receive
}

void loop() {
  loopCAN();  // Continually checks for incoming commands and sends telemetry

  // Just print when a command is received
  if (hasNewCommand()) {
    uint8_t cmd = getLastCommand();
    Serial.print("[CAN] Received Command: ");
    switch (cmd) {
      case 1: Serial.println("Home"); break;
      case 2: Serial.println("Extend Fully"); break;
      case 3: Serial.println("Retract Fully"); break;
      case 4: Serial.println("Midpoint"); break;
      default: Serial.println("Unknown Command"); break;
    }
  }

  // Print confirmation of sent telemetry every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.println("[CAN] Sent telemetry message.");
  }

}
