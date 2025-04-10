#include <driver/twai.h>

// ============================== STRUCTURE ==============================

/**
 * Structure matching the BMSMessage sent over the CAN bus.
 * Total payload size is 8 bytes.
 * Fields:
 *  - bms_id      : 1 byte - BMS node identifier
 *  - status      : 1 byte - System status (see enum definitions)
 *  - voltage     : 1 byte - Scaled voltage (0–255 = 0–25.5V)
 *  - current     : 1 byte - Scaled current (0–255 = 0–25.5A)
 *  - temp1_raw   : 2 bytes - Temperature 1 in hundredths of °C
 *  - temp2_raw   : 2 bytes - Temperature 2 in hundredths of °C
 */
struct __attribute__((packed)) BMSMessage {
  uint8_t  bms_id;
  uint8_t  status;
  uint8_t  voltage;
  uint8_t  current;
  uint16_t temp1_raw;
  uint16_t temp2_raw;
};

// ============================== STATUS DESCRIPTIONS ==============================

/**
 * Returns a human-readable string for a given BMS status code.
 *
 * @param status The numeric status code from the BMS
 * @return const char* description of the status
 */
const char* getStatusDescription(uint8_t status) {
  switch (status) {
    case 1: return "OK";
    case 2: return "INIT FAIL";
    case 3: return "OVERCURRENT";
    case 4: return "DISCHARGED";
    case 5: return "OVERTEMP";
    case 6: return "TEMP SENSOR FAIL";
    case 7: return "CURRENT SENSOR FAIL";
    default: return "UNKNOWN";
  }
}

// ============================== SETUP ==============================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 CAN Receiver (TX=4, RX=5)");

  // Configure TWAI (CAN) driver with specified TX/RX pins and normal operation mode
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)4, (gpio_num_t)5, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();              // Set CAN bus speed to 500 kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();            // Accept all CAN IDs

  Serial.println("Installing TWAI driver...");
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed.");
  } else {
    Serial.println("Failed to install TWAI driver.");
    while (1); // Stop execution on failure
  }

  Serial.println("Starting TWAI...");
  if (twai_start() == ESP_OK) {
    Serial.println("CAN bus started successfully.");
  } else {
    Serial.println("Failed to start CAN bus.");
    while (1); // Stop execution on failure
  }
}

// ============================== MAIN LOOP ==============================

void loop() {
  twai_message_t message;

  // Attempt to receive a CAN message with a 1-second timeout
  if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    // Validate message length matches expected BMS message size
    if (message.data_length_code == sizeof(BMSMessage)) {
      // Cast raw data buffer to BMSMessage structure
      BMSMessage* msg = (BMSMessage*)message.data;

      // Convert raw values to human-readable units
      float voltage = msg->voltage / 10.0;
      float current = msg->current / 10.0;
      float temp1 = msg->temp1_raw / 100.0;
      float temp2 = msg->temp2_raw / 100.0;

      // Output decoded information
      Serial.println("========== BMS STATUS ==========");
      Serial.print("CAN ID:      0x"); Serial.println(message.identifier, HEX);
      Serial.print("BMS ID:      "); Serial.println(msg->bms_id);
      Serial.print("Status:      "); Serial.println(getStatusDescription(msg->status));
      Serial.print("Voltage:     "); Serial.print(voltage); Serial.println(" V");
      Serial.print("Current:     "); Serial.print(current); Serial.println(" A");
      Serial.print("Temp 1:      "); Serial.print(temp1); Serial.println(" °C");
      Serial.print("Temp 2:      "); Serial.print(temp2); Serial.println(" °C");
      Serial.println("================================\n");
    } else {
      // Handle unexpected frame size
      Serial.print("Unexpected CAN frame size: ");
      Serial.println(message.data_length_code);
    }
  } else {
    // Timeout waiting for message
    Serial.println("No CAN message received.");
  }
}
