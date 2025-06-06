#include <driver/twai.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 CAN Receiver (TX=5, RX=4)");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.println("Installing TWAI driver...");
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed.");
  } else {
    Serial.println("❌ Failed to install TWAI driver.");
    while (1);
  }

  Serial.println("Starting TWAI...");
  if (twai_start() == ESP_OK) {
    Serial.println("✅ CAN started successfully!");
  } else {
    Serial.println("❌ Failed to start CAN.");
    while (1);
  }
}


void loop() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.print("Received CAN message - ID: 0x");
    Serial.print(message.identifier, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf("0x%02X ", message.data[i]);
    }
    Serial.println();
  } else {
    Serial.println("No CAN message received.");
  }
}





