#include <Arduino.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// === RS485 UART Setup (UART2) ===
HardwareSerial UART_RS485(2); // UART2 on ESP32
#define RS485_TX 9
#define RS485_RX 46

void setup() {
  Serial.begin(115200); // USB serial for debugging
  UART_RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX); // RS485 UART
  Serial.println("RS485 test initialized.");
}

void loop() {
  StaticJsonDocument<512> doc;

  doc["timestamp"] = millis();

  JsonObject bms1 = doc.createNestedObject("bms1");
  bms1["id"] = 1;
  bms1["status"] = "OK";
  bms1["voltage"] = 12.4;
  bms1["current"] = 1.2;
  bms1["temp1"] = 25.5;
  bms1["temp2"] = 26.0;
  bms1["timestamp"] = millis() - 20;

  JsonObject vehicle = doc.createNestedObject("vehicle");
  vehicle["pitch"] = 5.0;
  vehicle["roll"] = -3.2;
  vehicle["yaw"] = 180.0;
  vehicle["stateCode"] = 2;

  serializeJson(doc, UART_RS485); // Send over RS485
  UART_RS485.println();

  serializeJsonPretty(doc, Serial); // Print to USB serial for verification
  Serial.println();

  delay(1000); // Send every second
}
