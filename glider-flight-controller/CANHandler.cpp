#include "CANHandler.h"
#include <driver/twai.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Pins.h"

// ---- Structs ----

struct BMSData {
  bool active = false;
  uint8_t id = 0;
  uint8_t status = 0;
  float voltage = 0;
  float current = 0;
  float temp1 = 0;
  float temp2 = 0;
  unsigned long lastUpdate = 0;
};

struct VBDData {
  bool active = false;
  uint8_t id = 0;
  uint8_t status = 0;
  float position = 0.0f;  // in cm
  unsigned long lastUpdate = 0;
};

// ---- Arrays ----

BMSData bmsArray[4];     // BMS1–BMS4
VBDData vbdArray[2];     // VBD1–VBD2

// ---- Tasks ----

void CANReceiveTask(void* param) {
  while (true) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {

      // ---- BMS Data (0x100 – 0x103) ----
      if (msg.data_length_code == 8 && msg.identifier >= 0x100 && msg.identifier <= 0x103) {
        int idx = msg.identifier - 0x100;
        BMSData& b = bmsArray[idx];
        b.id = msg.data[0];
        b.status = msg.data[1];
        b.voltage = msg.data[2] / 10.0f;
        b.current = msg.data[3] / 10.0f;
        b.temp1 = (msg.data[4] | (msg.data[5] << 8)) / 100.0f;
        b.temp2 = (msg.data[6] | (msg.data[7] << 8)) / 100.0f;
        b.lastUpdate = millis();
        b.active = true;
      }

      else if (msg.data_length_code == 4 && msg.identifier >= 0x301 && msg.identifier <= 0x302) {
          int idx = msg.identifier - 0x301;  // 0x301→0, 0x302→1
          VBDData& v = vbdArray[idx];
          v.id       = msg.data[0];
          v.status   = msg.data[1];
          uint16_t rawPos = (msg.data[2] << 8) | msg.data[3];
          v.position = rawPos / 10.0f;
          v.lastUpdate = millis();
          v.active   = true;
      }
    }
  }
}

// ---- Setup ----

void initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }
}

void startCANReceiveTask() {
  xTaskCreatePinnedToCore(CANReceiveTask, "CANReceiveTask", 4096, NULL, 1, NULL, 1);
}

// ---- Accessors ----

void getLatestBMS(int index, JsonObject& obj) {
  if (index < 0 || index >= 4 || !bmsArray[index].active) return;
  BMSData& b = bmsArray[index];
  obj["id"] = b.id;
  obj["status"] = b.status;
  obj["voltage"] = b.voltage;
  obj["current"] = b.current;
  obj["temp1"] = b.temp1;
  obj["temp2"] = b.temp2;
  obj["timestamp"] = b.lastUpdate;
}

bool getLatestVBD(int index, JsonObject& obj) {
  if (index < 0 || index >= 2 || !vbdArray[index].active) return false;
  VBDData& v = vbdArray[index];
  obj["id"] = v.id;
  obj["status"] = v.status;
  obj["position"] = v.position;
  obj["timestamp"] = v.lastUpdate;
  return true;
}

// ---- Command Sender ----

void sendVBDCommand(uint8_t vbd_id, uint8_t command) {
  twai_message_t msg = {};
  msg.identifier = 0x210 + vbd_id;  // VBD1 = 0x210, VBD2 = 0x211
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 1;
  msg.data[0] = command;

  twai_transmit(&msg, pdMS_TO_TICKS(100));
}
