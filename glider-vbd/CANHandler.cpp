#include "CANHandler.h"
#include "Pins.h"
#include <driver/twai.h>

// ========== CONFIG ========== //
#define VBD_NODE_ID 1               // Change to 2 if this is VBD2
#define TELEMETRY_SEND_INTERVAL 1000  // ms

// ========== STATE ========== //
static uint8_t currentState = VBD_AWAITING_COMMAND;
static float currentPosition = 12.3;  // placeholder position

// ========== COMMAND STATE ========== //
static bool newCommandAvailable = false;
static uint8_t receivedCommand = 0;
static float targetPosition = 0;

// ========== TIMERS ========== //
static unsigned long lastTelemetrySent = 0;

// ========== SETUP ========== //
void initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("[CAN] Initialized");
  } else {
    Serial.println("[CAN] Init FAILED");
    while (true) delay(1000);
  }
}

// ========== MAIN LOOP HANDLER ========== //
void loopCAN() {
  // --- Handle RX ---
  twai_message_t rxMsg;
  if (twai_receive(&rxMsg, 0) == ESP_OK) {
    if (rxMsg.identifier == 0x200 + VBD_NODE_ID && rxMsg.data_length_code >= 1) {
      receivedCommand = rxMsg.data[0];
      newCommandAvailable = true;

      // Optional: decode target position if included
      if (rxMsg.data_length_code >= 3) {
        uint16_t raw = (rxMsg.data[1] << 8) | rxMsg.data[2];
        targetPosition = raw / 10.0f;
      }

      Serial.print("[CAN RX] Command: "); Serial.println(receivedCommand);
    }
  }

  // --- Handle TX ---
  if (millis() - lastTelemetrySent >= TELEMETRY_SEND_INTERVAL) {
    lastTelemetrySent = millis();

    twai_message_t txMsg = {};
    txMsg.identifier = 0x300 + VBD_NODE_ID;
    txMsg.extd = 0;
    txMsg.rtr = 0;
    txMsg.data_length_code = 4;

    txMsg.data[0] = VBD_NODE_ID;
    txMsg.data[1] = currentState;

    uint16_t positionRaw = (uint16_t)(currentPosition * 10.0f);  // 1 decimal place
    txMsg.data[2] = (positionRaw >> 8) & 0xFF;
    txMsg.data[3] = positionRaw & 0xFF;

    if (twai_transmit(&txMsg, pdMS_TO_TICKS(100)) == ESP_OK) {
      Serial.println("[CAN TX] Telemetry sent");
    } else {
      Serial.println("[CAN TX] Failed");
    }
  }
}

// ========== ACCESSORS ========== //
bool hasNewCommand() {
  return newCommandAvailable;
}

uint8_t getLastCommand() {
  newCommandAvailable = false;
  return receivedCommand;
}

float getTargetPosition() {
  return targetPosition;
}
