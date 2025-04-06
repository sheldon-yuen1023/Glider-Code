#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// === CONFIGURATION SECTION ===

// OneWire DS18B20 temperature sensor pin
#define ONE_WIRE_BUS 10  // GPIO10 (D10 on XIAO)

// INA228 I2C battery monitor configuration
#define INA228_ADDR 0x45           // I2C address of INA228
#define REG_BUS_VOLTAGE 0x05       // Register for bus voltage
#define REG_CURRENT 0x04           // Register for current
#define CURRENT_LSB 2.4414e-5      // Current LSB in Amps (204.8A full scale)
#define VOLTAGE_DIVIDER_RATIO 10.0 // Voltage divider ratio used on battery input

// I2C pin configuration for ESP32-C3 XIAO
#define I2C_SDA 6
#define I2C_SCL 7

// Power relay (DFRobot Gravity MOSFET control)
#define MOSFET_PIN 2  // GPIO2 controls MOSFET output relay

// Safety thresholds
#define CURRENT_LIMIT 15.0   // Max allowed current in Amps
#define VOLTAGE_MIN 0        // Min allowed voltage in Volts
#define TEMP_LIMIT_C 40.0    // Max allowed temperature in Celsius

// === GLOBAL VARIABLES ===

OneWire oneWire(ONE_WIRE_BUS);              // OneWire interface for DS18B20 sensors
DallasTemperature sensors(&oneWire);        // DallasTemperature library wrapper
float* temperatures = nullptr;              // Dynamic array to store temperature values
int sensorCount = 0;                        // Number of connected DS18B20 sensors

// Safety status flags
bool battery_overheated = false;
bool overcurrent = false;
bool battery_discharged = false;
bool ina_disconnected = false;
bool emergency_triggered = false;

// === SETUP FUNCTION ===

/**
 * setup()
 * Initializes all sensors and systems:
 * - Serial output
 * - I2C and OneWire interfaces
 * - Power relay control
 * - Sensor availability with timeout failsafe
 */
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  sensors.begin();

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);  // Turn relay ON at startup

  delay(500);
  Serial.println("Initializing system...");

  // Attempt to detect required sensors within 10 seconds
  unsigned long startTime = millis();
  bool tempReady = false;
  bool inaReady = false;

  while (millis() - startTime < 10000) {
    if (!tempReady && sensors.getDeviceCount() > 0) tempReady = true;
    if (!inaReady && isINA228Available()) inaReady = true;

    if (tempReady && inaReady) break;

    if (!tempReady) Serial.println("No DS18B20 temperature sensors detected.");
    if (!inaReady) Serial.println("INA228 battery monitor not responding on I2C.");
    delay(1000);
    sensors.begin();  // Retry OneWire init
  }

  // Failsafe: one or more sensors missing
  if (!tempReady || !inaReady) {
    Serial.println("\nFAILSAFE: Required sensors not detected after 10 seconds.");
    digitalWrite(MOSFET_PIN, LOW);  // Turn OFF relay
    while (true) delay(1000);       // Halt system
  }

  // Allocate temperature buffer and print sensor addresses
  sensorCount = sensors.getDeviceCount();
  temperatures = new float[sensorCount];

  Serial.print("DS18B20 sensor(s) detected: ");
  Serial.println(sensorCount);

  for (int i = 0; i < sensorCount; i++) {
    DeviceAddress addr;
    if (sensors.getAddress(addr, i)) {
      Serial.print("Sensor "); Serial.print(i); Serial.print(" address: ");
      printAddress(addr);
    } else {
      Serial.print("Sensor "); Serial.print(i); Serial.println(" address not found!");
    }
  }

  Serial.println("INA228 battery monitor detected.");
  Serial.println("System ready.");
  Serial.println("------------------------");
}

// === MAIN LOOP FUNCTION ===

/**
 * loop()
 * Continuously monitors:
 * - Temperatures from DS18B20 sensors
 * - Battery voltage and current from INA228
 * - INA228 presence (failsafe)
 * Responds to any fault by:
 * - Disabling relay
 * - Printing cause of emergency
 */
void loop() {
  // If already in emergency mode, repeatedly print cause
  if (emergency_triggered) {
    reportEmergencyCause();
    delay(2000);
    return;
  }

  // Check for I2C disconnect of INA228 during operation
  if (!isINA228Available()) {
    ina_disconnected = true;
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);
    Serial.println("EMERGENCY CONDITION DETECTED! INA228 disconnected. Power disabled.");
    return;
  }

  updateTemperatures();        // Read DS18B20 sensors
  updatePowerReadings();       // Read INA228 voltage & current
  evaluateSafetyConditions();  // Check thresholds

  if (battery_overheated || battery_discharged || overcurrent) {
    emergency_triggered = true;
    digitalWrite(MOSFET_PIN, LOW);  // Disable relay
    Serial.println("EMERGENCY CONDITION DETECTED! Power disabled.");
    return;
  }

  reportSystemStatus();  // Normal status report
  delay(2000);
}

// === MONITORING MODULES ===

/**
 * updateTemperatures()
 * Requests temperature readings from each connected sensor
 * and stores in the global `temperatures[]` array.
 */
void updateTemperatures() {
  sensors.requestTemperatures();
  for (int i = 0; i < sensorCount; i++) {
    float temp = sensors.getTempCByIndex(i);
    temperatures[i] = temp;
    Serial.print("Temp"); Serial.print(i + 1); Serial.print(": ");
    Serial.print(temp); Serial.println(" °C");
  }
}

/**
 * updatePowerReadings()
 * Reads battery voltage and current from INA228,
 * and updates the `overcurrent` and `battery_discharged` flags.
 */
void updatePowerReadings() {
  float voltage = readBusVoltage();
  float current = readCurrent();

  battery_discharged = (voltage < VOLTAGE_MIN);
  overcurrent = (current > CURRENT_LIMIT);

  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current); Serial.println(" A");
}

/**
 * evaluateSafetyConditions()
 * Checks the temperature array for overtemperature or disconnected sensors.
 * Sets `battery_overheated` flag if any issues are detected.
 */
void evaluateSafetyConditions() {
  battery_overheated = false;
  for (int i = 0; i < sensorCount; i++) {
    if (temperatures[i] >= TEMP_LIMIT_C || temperatures[i] == DEVICE_DISCONNECTED_C) {
      battery_overheated = true;
    }
  }
}

/**
 * reportSystemStatus()
 * Prints normal system status when all parameters are within safe limits.
 */
void reportSystemStatus() {
  Serial.println("System status: OK");
  Serial.println("------------------------");
}

/**
 * reportEmergencyCause()
 * Reports which fault condition triggered the emergency shutdown.
 */
void reportEmergencyCause() {
  Serial.println("SYSTEM IN EMERGENCY STATE:");
  if (battery_overheated) Serial.println("- Overtemperature detected.");
  if (battery_discharged) Serial.println("- Battery voltage too low.");
  if (overcurrent)        Serial.println("- Overcurrent condition.");
  if (ina_disconnected)   Serial.println("- INA228 monitor not responding.");
  Serial.println("Power is disabled. Reset board to restart system.");
  Serial.println("------------------------");
}

// === INA228 INTERFACE FUNCTIONS ===

/**
 * isINA228Available()
 * Sends a transmission to the INA228.
 * Returns true if device responds, false if not.
 */
bool isINA228Available() {
  Wire.beginTransmission(INA228_ADDR);
  return (Wire.endTransmission() == 0);
}

/**
 * readINA228Register24()
 * Reads a 24-bit register value from INA228 (unsigned).
 */
uint32_t readINA228Register24(uint8_t reg) {
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA228_ADDR, (uint8_t)3);

  uint32_t value = 0;
  if (Wire.available() == 3) {
    value = (Wire.read() << 16);
    value |= (Wire.read() << 8);
    value |= Wire.read();
  }
  return value;
}

/**
 * readINA228RegisterSigned24()
 * Reads a 24-bit signed register from INA228 and extends it to 32-bit int.
 */
int32_t readINA228RegisterSigned24(uint8_t reg) {
  uint32_t value = readINA228Register24(reg);
  if (value & 0x800000) {
    value |= 0xFF000000;  // Sign extend
  }
  return (int32_t)value;
}

/**
 * readBusVoltage()
 * Reads bus voltage and scales according to LSB and voltage divider.
 */
float readBusVoltage() {
  uint32_t rawVoltage = readINA228Register24(REG_BUS_VOLTAGE);
  float voltage = rawVoltage * 1.25e-6;  // Each LSB = 1.25 µV
  voltage *= VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

/**
 * readCurrent()
 * Reads current value in Amps from INA228.
 */
float readCurrent() {
  int32_t rawCurrent = readINA228RegisterSigned24(REG_CURRENT);
  float current = rawCurrent * CURRENT_LSB;
  return current;
}

// === HELPER FUNCTION ===

/**
 * printAddress()
 * Prints an 8-byte DS18B20 ROM address in hex.
 */
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}
