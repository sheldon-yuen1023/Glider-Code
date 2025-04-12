#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/twai.h>
#include <HardwareSerial.h>

// ========== CAN (TWAI) Config ==========
#define CAN_RX GPIO_NUM_5
#define CAN_TX GPIO_NUM_4

// ========== RS485 UART Config ==========
HardwareSerial UART_RS485(2); // Use UART2
#define RS485_TX 17
#define RS485_RX 18

// ========== BMS Message Structure ==========
struct __attribute__((packed)) BMSMessage {
  uint8_t  bms_id;
  uint8_t  status;
  uint8_t  voltage;
  uint8_t  current;
  uint16_t temp1_raw;
  uint16_t temp2_raw;
};

// ========== Human-Readable Status ==========
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

// ========== Decoded BMS Data ==========
struct DecodedBMS {
  bool active = false;
  uint8_t bms_id = 0;
  uint8_t status = 0;
  float voltage = 0;
  float current = 0;
  float temp1 = 0;
  float temp2 = 0;
  unsigned long lastUpdate = 0;
};

DecodedBMS bmsData[4];

// ========== Dummy IMU Placeholders ==========
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32-S3 CAN-to-RS485 JSON Aggregator");

  // === Init CAN (TWAI) ===
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("Failed to initialize CAN.");
    while (1);
  }
  Serial.println("CAN bus started on GPIO 4/5.");

  // === Init RS485 UART (UART2) ===
  UART_RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("RS485 UART initialized on GPIO 17/18.");
}

void loop() {
  // === Listen for CAN messages ===
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK &&
      message.data_length_code == sizeof(BMSMessage)) {

    BMSMessage* msg = (BMSMessage*)message.data;

    int index = -1;
    if (message.identifier >= 0x101 && message.identifier <= 0x104) {
      index = message.identifier - 0x101;
    }

    if (index >= 0 && index < 4) {
      DecodedBMS& bms = bmsData[index];
      bms.bms_id = msg->bms_id;
      bms.status = msg->status;
      bms.voltage = msg->voltage / 10.0;
      bms.current = msg->current / 10.0;
      bms.temp1 = msg->temp1_raw / 100.0;
      bms.temp2 = msg->temp2_raw / 100.0;
      bms.lastUpdate = millis();
      bms.active = true;
    }
  }

  // === Send JSON snapshot every second ===
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 1000) {
    sendJSON();
    lastSend = millis();
  }
}

void sendJSON() {
  StaticJsonDocument<1024> doc;

  JsonObject bms = doc.createNestedObject("bms");

  for (int i = 0; i < 4; i++) {
    if (bmsData[i].active) {
      String key = "bms" + String(i + 1);
      JsonObject obj = bms.createNestedObject(key);
      obj["id"] = bmsData[i].bms_id;
      obj["status"] = getStatusDescription(bmsData[i].status);
      obj["voltage"] = bmsData[i].voltage;
      obj["current"] = bmsData[i].current;
      obj["temp1"] = bmsData[i].temp1;
      obj["temp2"] = bmsData[i].temp2;
      obj["lastUpdate"] = bmsData[i].lastUpdate;
    }
  }

  JsonObject vehicle = doc.createNestedObject("vehicle");
  vehicle["state"] = "idle";
  vehicle["pitch"] = pitch;
  vehicle["roll"] = roll;
  vehicle["yaw"] = yaw;

  // Send over RS485 (UART2)
  serializeJson(doc, UART_RS485);
  UART_RS485.println();

  // Also print to Serial (USB) for debug
  serializeJsonPretty(doc, Serial);
  Serial.println();
}
