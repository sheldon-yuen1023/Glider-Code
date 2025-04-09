/**
 * Battery Management System with CAN Broadcast
 * - Reads battery voltage and current via INA228 (I2C)
 * - Reads temperatures via DS18B20 (1-Wire)
 * - Evaluates safety conditions (overtemp, overcurrent, undervoltage)
 * - Sends compact status packets over CAN bus using ESP32-C3 TWAI (TX=21, RX=20)
 * 
 * !!! MAKE SURE THAT VOLTAGE AND TEMPERATURE FAILSAFE VALUES ARE SET CORRECTLY BEFORE FLASHING !!!
 */

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/twai.h>  // TWAI = CAN protocol driver

// ============================== CONFIGURATION ==============================

// I2C pins for INA228 battery monitor
#define I2C_SDA 6
#define I2C_SCL 7

// INA228 config
#define INA228_ADDR 0x45
#define REG_BUS_VOLTAGE 0x05
#define REG_CURRENT 0x04
#define CURRENT_LSB 2.4414e-5      // 204.8A full scale, 2.44e-5 A/bit
#define VOLTAGE_DIVIDER_RATIO 10.0 // External voltage divider factor

// 1-Wire pin for DS18B20 sensors
#define ONE_WIRE_BUS 10  // GPIO10

// Relay (MOSFET control) pin
#define MOSFET_PIN 2     // GPIO2

// CAN bus pins
#define CAN_TX_PIN 21
#define CAN_RX_PIN 20

// Safety thresholds
#define CURRENT_LIMIT 15.0       // Amps
#define VOLTAGE_MIN 0            // Volts
#define TEMP_LIMIT_C 40.0        // Celsius
#define BMS_NODE_ID 1            // Unique ID for this BMS

// ============================== STRUCTURES ==============================

/**
 * Compact structure for BMS status broadcast over CAN.
 * Total size = 8 bytes
 */
struct __attribute__((packed)) BMSMessage {
  uint8_t bms_id;         // 1 byte - Unique BMS ID
  uint8_t status;         // 1 byte - 0=OFF, 1=ON
  uint8_t voltage;        // 1 byte - 0–255 = 0–25.5V
  uint8_t current;        // 1 byte - 0–255 = 0–25.5A
  uint16_t temp1_raw;     // 2 bytes - Temperature * 100 (0.00–40.00°C)
  uint16_t temp2_raw;     // 2 bytes - Temperature * 100
};

// ============================== GLOBAL OBJECTS ==============================

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float* temperatures = nullptr;
int sensorCount = 0;

// Safety status flags
bool battery_overheated = false;
bool overcurrent = false;
bool battery_discharged = false;
bool ina_disconnected = false;
bool emergency_triggered = false;

// ============================== SETUP ==============================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing BMS Node...");

  // Initialize I2C, 1-Wire, and power relay
  Wire.begin(I2C_SDA, I2C_SCL);
  sensors.begin();
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);  // Turn relay ON by default

  // Sensor detection with timeout
  unsigned long startTime = millis();
  bool tempReady = false, inaReady = false;
  while (millis() - startTime < 10000) {
    if (!tempReady && sensors.getDeviceCount() > 0) tempReady = true;
    if (!inaReady && isINA228Available()) inaReady = true;
    if (tempReady && inaReady) break;
    delay(1000);
    sensors.begin();  // Reinit DS18B20 if needed
  }

  if (!tempReady || !inaReady) {
    Serial.println("FAILSAFE: Sensors not detected. Disabling power.");
    digitalWrite(MOSFET_PIN, LOW);
    while (true) delay(1000);
  }

  sensorCount = sensors.getDeviceCount();
  temperatures = new float[sensorCount];

  Serial.print("DS18B20 count: "); Serial.println(sensorCount);
  Serial.println("INA228 detected.");
  Serial.println("System ready.\n");

  // Setup CAN bus
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("ERROR: Failed to initialize CAN bus.");
    while (true) delay(1000);
  }

  Serial.println("CAN bus initialized.\n");
}

// ============================== MAIN LOOP ==============================

void loop() {
  if (emergency_triggered) {
    reportEmergencyCause();
    delay(2000);
    return;
  }

  if (!isINA228Available()) {
    ina_disconnected = true;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    Serial.println("EMERGENCY: INA228 disconnected. Power disabled.");
    return;
  }

  updateTemperatures();
  updatePowerReadings();
  evaluateSafetyConditions();

  if (battery_overheated || battery_discharged || overcurrent) {
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    Serial.println("EMERGENCY: Safety threshold breached. Power disabled.");
    return;
  }

  reportSystemStatus();
  sendCANStatusFrame();  // Send CAN frame with system data

  delay(2000);  // Adjust as needed
}

// ============================== MODULES ==============================

void updateTemperatures() {
  sensors.requestTemperatures();
  for (int i = 0; i < sensorCount; i++) {
    float temp = sensors.getTempCByIndex(i);
    temperatures[i] = temp;
    Serial.print("Temp"); Serial.print(i); Serial.print(": ");
    Serial.print(temp); Serial.println(" °C");
  }
}

void updatePowerReadings() {
  float voltage = readBusVoltage();
  float current = readCurrent();

  battery_discharged = (voltage < VOLTAGE_MIN);
  overcurrent = (current > CURRENT_LIMIT);

  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current); Serial.println(" A");
}

void evaluateSafetyConditions() {
  battery_overheated = false;
  for (int i = 0; i < sensorCount; i++) {
    if (temperatures[i] >= TEMP_LIMIT_C || temperatures[i] == DEVICE_DISCONNECTED_C) {
      battery_overheated = true;
    }
  }
}

void reportSystemStatus() {
  Serial.println("System status: OK\n--------------------");
}

void reportEmergencyCause() {
  Serial.println("SYSTEM IN EMERGENCY STATE:");
  if (battery_overheated) Serial.println("- Overtemperature detected.");
  if (battery_discharged) Serial.println("- Battery voltage too low.");
  if (overcurrent)        Serial.println("- Overcurrent condition.");
  if (ina_disconnected)   Serial.println("- INA228 disconnected.");
  Serial.println("Power is disabled.\n--------------------");
}

// ============================== CAN TRANSMISSION ==============================

void sendCANStatusFrame() {
  BMSMessage msg;
  msg.bms_id = BMS_NODE_ID;
  msg.status = emergency_triggered ? 0 : 1;
  msg.voltage = (uint8_t)(readBusVoltage() * 10.0);      // 12.3V → 123
  msg.current = (uint8_t)(readCurrent() * 10.0);         // 4.2A → 42
  msg.temp1_raw = (uint16_t)(temperatures[0] * 100.0);   // 24.56°C → 2456
  msg.temp2_raw = (sensorCount > 1) ? (uint16_t)(temperatures[1] * 100.0) : 0;

  twai_message_t message;
  message.identifier = 0x100 + msg.bms_id;
  message.extd = 0;  // Standard frame
  message.rtr = 0;   // Data frame
  message.data_length_code = sizeof(BMSMessage);
  memcpy(message.data, &msg, sizeof(BMSMessage));

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN BMS message sent.");
  } else {
    Serial.println("Failed to send CAN message.");
  }
}

// ============================== INA228 UTILITIES ==============================

bool isINA228Available() {
  Wire.beginTransmission(INA228_ADDR);
  return (Wire.endTransmission() == 0);
}

uint32_t readINA228Register24(uint8_t reg) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA228_ADDR, (uint8_t)3);

  uint32_t value = 0;
  if (Wire.available() == 3) {
    value = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
  }
  return value;
}

int32_t readINA228RegisterSigned24(uint8_t reg) {
  uint32_t value = readINA228Register24(reg);
  if (value & 0x800000) value |= 0xFF000000;  // Sign extend
  return (int32_t)value;
}

float readBusVoltage() {
  uint32_t raw = readINA228Register24(REG_BUS_VOLTAGE);
  return raw * 1.25e-6 * VOLTAGE_DIVIDER_RATIO;
}

float readCurrent() {
  int32_t raw = readINA228RegisterSigned24(REG_CURRENT);
  return raw * CURRENT_LSB;
}

// ============================== DEBUG UTILITY ==============================

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
