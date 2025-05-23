/**
 * Battery Management System with CAN Broadcast
 * - Reads battery voltage and current via INA228 (I2C)
 * - Reads temperatures via DS18B20 (1-Wire)
 * - Evaluates safety conditions (init or runtime)
 * - Sends compact status packets over CAN bus using ESP32-C3 TWAI (TX=21, RX=20)
 *
 * !!! MAKE SURE THAT VOLTAGE AND TEMPERATURE FAILSAFE VALUES ARE SET CORRECTLY BEFORE FLASHING !!!
 */

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/twai.h>  // TWAI = CAN protocol driver

// ============================== CONFIGURATION ==============================

// Pin mappings and sensor-specific parameters
#define I2C_SDA 6                      // I2C SDA pin
#define I2C_SCL 7                      // I2C SCL pin
#define INA228_ADDR 0x45              // INA228 I2C address
#define REG_BUS_VOLTAGE 0x05          // INA228 voltage register
#define REG_CURRENT 0x04              // INA228 current register
#define CURRENT_LSB 2.4414e-5         // Conversion factor for current (A/bit)
#define VOLTAGE_DIVIDER_RATIO 10.0    // Hardware voltage divider ratio
#define ONE_WIRE_BUS 8                // DS18B20 1-Wire data pin
#define MOSFET_PIN 5                  // GPIO to control MOSFET/relay
#define CAN_TX_PIN 21                 // CAN TX pin
#define CAN_RX_PIN 20                 // CAN RX pin

// Safety limits
#define CURRENT_LIMIT 15.0            // Max allowable current (A)
#define VOLTAGE_MIN 0                 // Minimum safe voltage (V)
#define TEMP_LIMIT_C 40.0             // Maximum safe temperature (°C)
#define BMS_NODE_ID 3                 // Unique BMS identifier on CAN bus

// Timers for update loops
#define SENSOR_UPDATE_INTERVAL 1000   // Time between sensor reads (ms)
#define CAN_SEND_INTERVAL 5000        // Time between CAN broadcasts (ms)

// ============================== STATUS ENUM ==============================

/**
 * Enumerated list of BMS error/status codes for internal logic and CAN transmission.
 */
enum BMSStatus : uint8_t {
  STATUS_OK = 1,                      // System operating normally
  STATUS_INIT_FAIL = 2,              // Startup/init failure of sensors
  STATUS_OVERCURRENT = 3,            // Current exceeded safe threshold
  STATUS_DISCHARGED = 4,             // Voltage below minimum safe level
  STATUS_OVERTEMP = 5,               // One or more temperatures exceeded limit
  STATUS_TEMPERATURE_SENSOR_FAIL = 6, // Temperature sensors not responding
  STATUS_CURRENT_SENSOR_FAIL = 7     // INA228 not responding
};

// ============================== GLOBALS ==============================

unsigned long lastSensorUpdateTime = 0;     // Timestamp of last sensor read
unsigned long lastCANSendTime = 0;          // Timestamp of last CAN message
bool emergency_triggered = false;           // Emergency latch state
bool emergency_reported = false;            // Ensure only one emergency report is printed
uint8_t current_status = STATUS_OK;         // System status tracker

OneWire oneWire(ONE_WIRE_BUS);              // 1-Wire bus instance
DallasTemperature sensors(&oneWire);        // DallasTemperature sensor object
float* temperatures = nullptr;              // Pointer to array of temperature readings
int sensorCount = 0;                        // Number of detected DS18B20 sensors

// Structure used to encode and transmit BMS status over CAN (8 bytes total)
struct __attribute__((packed)) BMSMessage {
  uint8_t bms_id;
  uint8_t status;
  uint8_t voltage;
  uint8_t current;
  uint16_t temp1_raw;
  uint16_t temp2_raw;
};

// ============================== SETUP ==============================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing BMS Node...");

  // Begin communication with sensors and peripherals
  Wire.begin(I2C_SDA, I2C_SCL);
  sensors.begin();
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH); // Enable power relay initially

  // Basic presence check on sensors
  bool tempReady = sensors.getDeviceCount() > 0;
  bool inaReady = isINA228Available();

  if (!tempReady || !inaReady) {
    Serial.println("INIT FAILURE:");
    if (!tempReady) Serial.println("- No DS18B20 temperature sensors detected.");
    if (!inaReady) Serial.println("- INA228 current sensor not responding.");
    current_status = STATUS_INIT_FAIL;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW); // Disable power relay immediately
  } else {
    sensorCount = sensors.getDeviceCount();
    temperatures = new float[sensorCount];
  }

  // Initialize CAN bus
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
    digitalWrite(MOSFET_PIN, LOW);  // Ensure power remains off
    reportEmergencyCause();         // Print failure reason once

    // Transmit fault status over CAN at regular interval
    if (millis() - lastCANSendTime >= CAN_SEND_INTERVAL) {
      lastCANSendTime = millis();
      sendCANStatusFrame();
    }

    delay(1000);  // Avoid spamming serial output
    return;
  }

  unsigned long currentTime = millis();

  // Periodic sensor updates
  if (currentTime - lastSensorUpdateTime >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdateTime = currentTime;
    updateTemperatures();
    updatePowerReadings();
    evaluateSystemHealth();  // Unified health check
    if (!emergency_triggered) reportSystemStatus();
  }

  // Periodic CAN transmission
  if (currentTime - lastCANSendTime >= CAN_SEND_INTERVAL) {
    lastCANSendTime = currentTime;
    sendCANStatusFrame();
  }
}

// ============================== SAFETY & MONITORING ==============================

/**
 * Unified system health evaluator.
 * Evaluates sensor presence and data limits and sets system emergency state.
 */
void evaluateSystemHealth() {
  // Do not re-evaluate if already in emergency state
  if (emergency_triggered) return;
  
  // Verify INA228 is still responsive
  if (!isINA228Available()) {
    current_status = STATUS_CURRENT_SENSOR_FAIL;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    return;
  }

  // Re-check for temperature sensors
  sensorCount = sensors.getDeviceCount();
  if (sensorCount == 0) {
    current_status = STATUS_TEMPERATURE_SENSOR_FAIL;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    return;
  }

  // Loop through each temperature and check thresholds
  for (int i = 0; i < sensorCount; i++) {
    if (temperatures[i] >= TEMP_LIMIT_C || temperatures[i] == DEVICE_DISCONNECTED_C) {
      current_status = STATUS_OVERTEMP;
      emergency_triggered = true;
      digitalWrite(MOSFET_PIN, LOW);
      return;
    }
  }

  // Read power data and check thresholds
  float voltage = readBusVoltage();
  float current = readCurrent();

  if (voltage < VOLTAGE_MIN) {
    current_status = STATUS_DISCHARGED;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    return;
  }
  if (current > CURRENT_LIMIT) {
    current_status = STATUS_OVERCURRENT;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    return;
  }

  // System is healthy
  current_status = STATUS_OK;
}

/**
 * Request and store temperatures from DS18B20 sensors
 */
void updateTemperatures() {
  sensors.requestTemperatures();
  for (int i = 0; i < sensorCount; i++) {
    float temp = sensors.getTempCByIndex(i);
    temperatures[i] = temp;
    Serial.print("Temp"); Serial.print(i); Serial.print(": ");
    Serial.print(temp); Serial.println(" °C");
  }
}

/**
 * Read and print voltage and current values
 */
void updatePowerReadings() {
  float voltage = readBusVoltage();
  float current = readCurrent();
  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current); Serial.println(" A");
}

// ============================== STATUS ==============================

/**
 * Print normal operation status
 */
void reportSystemStatus() {
  Serial.println("System status: OK\n--------------------");
}

/**
 * Print reason for system shutdown
 */
void reportEmergencyCause() {
  if (emergency_reported) return;
  emergency_reported = true;
  Serial.println("SYSTEM IN EMERGENCY STATE:");
  switch (current_status) {
    case STATUS_INIT_FAIL: Serial.println("- Sensor initialization failure."); break;
    case STATUS_OVERCURRENT: Serial.println("- Overcurrent condition."); break;
    case STATUS_DISCHARGED: Serial.println("- Battery undervoltage."); break;
    case STATUS_OVERTEMP: Serial.println("- Overtemperature detected."); break;
    case STATUS_TEMPERATURE_SENSOR_FAIL: Serial.println("- Lost temperature sensor(s)."); break;
    case STATUS_CURRENT_SENSOR_FAIL: Serial.println("- INA228 disconnected."); break;
    default: Serial.println("- Unknown emergency state."); break;
  }
  Serial.println("Power is disabled.\n--------------------");
}

// ============================== CAN ==============================

/**
 * Compose and send BMS status frame over CAN
 */
void sendCANStatusFrame() {
  BMSMessage msg;
  
  float voltage = 0.0;
  float current = 0.0;

  if (current_status != STATUS_CURRENT_SENSOR_FAIL && current_status != STATUS_INIT_FAIL) {
    voltage = readBusVoltage();
    current = readCurrent();
  }

  msg.voltage = (uint8_t)(voltage * 10.0);   // Encode voltage 0–25.5V
  msg.current = (uint8_t)(current * 10.0);   // Encode current 0–25.5A
  
  msg.bms_id = BMS_NODE_ID;
  msg.status = current_status;
msg.temp1_raw = (temperatures != nullptr && sensorCount > 0) ? (uint16_t)(temperatures[0] * 100.0) : 0;
msg.temp2_raw = (temperatures != nullptr && sensorCount > 1) ? (uint16_t)(temperatures[1] * 100.0) : 0;


  twai_message_t message;
  message.identifier = 0x100 + msg.bms_id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = sizeof(BMSMessage);
  memcpy(message.data, &msg, sizeof(BMSMessage));

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.println("CAN BMS message sent.");
  } else {
    Serial.println("Failed to send CAN message.");
  }
}

// ============================== INA ==============================

/**
 * Check if INA228 sensor responds to I2C
 */
bool isINA228Available() {
  Wire.beginTransmission(INA228_ADDR);
  return (Wire.endTransmission() == 0);
}

/**
 * Read a 24-bit unsigned register from INA228
 */
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

/**
 * Read a 24-bit signed register and convert to 32-bit signed value
 */
int32_t readINA228RegisterSigned24(uint8_t reg) {
  uint32_t value = readINA228Register24(reg);
  if (value & 0x800000) value |= 0xFF000000;
  return (int32_t)value;
}

/**
 * Convert raw bus voltage from INA228 to volts
 */
float readBusVoltage() {
  uint32_t raw = readINA228Register24(REG_BUS_VOLTAGE);
  return raw * 1.25e-6 * VOLTAGE_DIVIDER_RATIO;
}

/**
 * Convert raw current from INA228 to amps
 */
float readCurrent() {
  int32_t raw = readINA228RegisterSigned24(REG_CURRENT);
  return raw * CURRENT_LSB;
}

// ============================== DEBUG ==============================

/**
 * Print sensor address in HEX (optional utility)
 */
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
