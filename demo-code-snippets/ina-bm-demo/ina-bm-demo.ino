#include <Wire.h>

#define INA228_ADDR 0x45  // Found from scanner
#define REG_BUS_VOLTAGE 0x05  // Bus voltage register

void setup() {
  Wire.begin(6, 7);  // ESP32-C3 SDA = GPIO6, SCL = GPIO7
  Serial.begin(115200);
  delay(1000);
  Serial.println("INA228 Reader");
}

void loop() {
  uint32_t rawVoltage = readINA228Register24(REG_BUS_VOLTAGE);
  
  // Convert per INA228 datasheet: 1 LSB = 1.25 µV
  float voltage = rawVoltage * 1.25e-6 *10;

  Serial.print("Bus Voltage: ");
  Serial.print(voltage, 6);
  Serial.println(" V");

  delay(1000);
}

// Reads a 24-bit register from INA228
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
