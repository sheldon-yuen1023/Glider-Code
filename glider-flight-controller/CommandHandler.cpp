#include "CommandHandler.h"
#include <Arduino.h>
#include "Pins.h"
#include "CANHandler.h"

extern HardwareSerial RS485;

// Function to handle the command string
void handleCommand(String cmd) {
  Serial.print("[RS485 CMD] ");
  Serial.println(cmd);  // Debug: show raw command

  if (cmd.startsWith("VBD,")) {
    String arg = cmd.substring(4);
    uint8_t command = 0;

    if (arg == "IN")      command = 1;
    else if (arg == "MID")  command = 2;
    else if (arg == "OUT")  command = 3;
    else if (arg == "ZERO") command = 4;
    else return;  // Unknown command

    // Send to both VBDs
    sendVBDCommand(0, command);  // VBD1
    Serial.printf("[DEBUG] Forwarded VBD command %d to VBD1\n", command);

    sendVBDCommand(1, command);  // VBD2
    Serial.printf("[DEBUG] Forwarded VBD command %d to VBD2\n", command);
  }

  // TODO: Handle other command types here
}

void CommandReceiveTask(void* param) {
  String input;

  while (true) {
    while (RS485.available()) {
      char c = RS485.read();
      if (c == '\n') {
        input.trim();
        if (!input.isEmpty()) {
          handleCommand(input);  // <--- key addition
        }
        input = "";
      } else {
        input += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void startCommandReceiveTask() {
  xTaskCreatePinnedToCore(CommandReceiveTask, "CommandReceiveTask", 4096, NULL, 1, NULL, 1);
}
