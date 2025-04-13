#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/twai.h>
#include <HardwareSerial.h>


/*
Example telemetry JSON frame:

{
  "timestamp": 12345678,
  "bms": {
    "bms1": {
      "id": 1,
      "status": "OK",
      "voltage": 12.3,
      "current": 1.4,
      "temp1": 25.6,
      "temp2": 26.0,
      "timestamp": 12345555
    },
    "bms2": {
      "id": 2,
      "status": "OVERTEMP",
      "voltage": 11.9,
      "current": 1.7,
      "temp1": 45.3,
      "temp2": 44.8,
      "timestamp": 12345556
    }
  },
  "vehicle": {
    "stateCode": 3,
    "pitch": 5.8,
    "roll": -2.4,
    "yaw": 182.0
  },
  "sensors": {
    "pressure": 98.6,
    "distanceToBottom": 3.4,
    "verticalVelocity": -0.12,
    "horizontalVelocity": 0.75,
    "leakSensors": {
      "sensor1": false,
      "sensor2": false,
      "sensor3": true
    }
  },
  "actuators": {
    "vbd1Position": 42.1,
    "vbd2Position": 38.6,
    "pitchPosition": 12.5,
    "rollPosition": 3.7
  }
}




*/


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

// ========== Dummy telemetry Placeholders ==========
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;
int currentSystemStateCode = 3;  // Example
float pressureReading = 98.6;
float sonarDistance = 2.8;
float verticalVelocity = -0.12;
float horizontalVelocity = 0.75;
bool leak1 = false;
bool leak2 = false;
bool leak3 = false;
float VBD1_position = 0.0;      // mm
float VBD2_position = 0.0;      // mm
float Pitch_position = 0.0;     // mm (linear actuator for pitch trim)
float Roll_position = 0.0;      // degrees (rotational actuator for roll)


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

  // Timestamp for the whole packet
  doc["timestamp"] = millis();  // Replace with RTC timestamp if available

  // Battery Monitoring Systems (BMS)
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
      obj["timestamp"] = bmsData[i].lastUpdate;
    }
  }

  // Vehicle state and orientation
  JsonObject vehicle = doc.createNestedObject("vehicle");
  vehicle["stateCode"] = currentSystemStateCode;  // Integer for surface-side decoding
  vehicle["pitch"] = pitch;
  vehicle["roll"] = roll;
  vehicle["yaw"] = yaw;

  // Sensors and environment
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["pressure"] = pressureReading;  // in dbar or mbar
  sensors["distanceToBottom"] = sonarDistance;  // in meters
  sensors["verticalVelocity"] = verticalVelocity;  // in m/s
  sensors["horizontalVelocity"] = horizontalVelocity;  // optional

  // Leak sensors
  JsonObject leak = sensors.createNestedObject("leakSensors");
  leak[sensor1] = 0;
  leak[sensor2] = 0;
  leak[sensor3] = 0;

    // Actuator positions
  JsonObject actuators = doc.createNestedObject("actuators");
  actuators["vbd1Position"] = VBD1_position;
  actuators["vbd2Position"] = VBD2_position;
  actuators["pitchPosition"] = Pitch_position;
  actuators["rollPosition"] = Roll_position;


  // Send over RS485 (UART2)
  serializeJson(doc, UART_RS485);
  UART_RS485.println();

  // Also print to Serial (USB) for debug
  serializeJsonPretty(doc, Serial);
  Serial.println();
}
