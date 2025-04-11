#include <driver/twai.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-C3 CAN Transmitter (TX=21, RX=20)");

  // Configure TWAI with your pin choices
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)21, (gpio_num_t)20, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed.");
  } else {
    Serial.println("Failed to install TWAI driver.");
    while (1);
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN started successfully!");
  } else {
    Serial.println("Failed to start CAN.");
    while (1);
  }
}

void loop() {
  twai_message_t message;
  message.identifier = 0x123;
  message.extd = 0; // Standard frame
  message.rtr = 0;  // Data frame
  message.data_length_code = 4;
  message.data[0] = 0xDE;
  message.data[1] = 0xAD;
  message.data[2] = 0xBE;
  message.data[3] = 0xEF;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN message sent!");
  } else {
    Serial.println("Failed to send CAN message.");
  }

  delay(1000);
}
