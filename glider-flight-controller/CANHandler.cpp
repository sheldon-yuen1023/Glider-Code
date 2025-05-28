#include "CANHandler.h"
#include <driver/twai.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Pins.h"

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

BMSData bmsArray[5];

void CANReceiveTask(void* param) {
  while (true) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (msg.data_length_code == 8 && msg.identifier >= 0x100 && msg.identifier <= 0x104) {
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
    }
  }
}

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

void getLatestBMS(int index, JsonObject& obj) {
  if (index < 0 || index >= 5 || !bmsArray[index].active) return;
  BMSData& b = bmsArray[index];
  obj["id"] = b.id;
  obj["status"] = b.status;
  obj["voltage"] = b.voltage;
  obj["current"] = b.current;
  obj["temp1"] = b.temp1;
  obj["temp2"] = b.temp2;
  obj["timestamp"] = b.lastUpdate;
}
