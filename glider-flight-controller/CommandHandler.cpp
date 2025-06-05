#include "CommandHandler.h"
#include <Arduino.h>
#include "Pins.h"

extern HardwareSerial RS485;

void CommandReceiveTask(void* param) {
  String input;

  while (true) {
    while (RS485.available()) {
      char c = RS485.read();
      if (c == '\n') {
        input.trim();
        if (!input.isEmpty()) {
          Serial.print("[RS485 CMD] ");
          Serial.println(input);
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
