#include <HardwareSerial.h>

// Use UART1 (you can also try UART2 if needed)
HardwareSerial UART_RS485(1);

// Updated pin assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

void setup() {
  // Start USB serial for debug
  Serial.begin(115200);
  delay(1000);

  // Start UART using GPIO 4 (TX) and 5 (RX)
  UART_RS485.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.println("UART_RS485 initialized on GPIO 4 (TX), 5 (RX)");
}

void loop() {
  UART_RS485.println("Hello from ESP32 on GPIO 4/5!");
  Serial.println("Sent message over UART_RS485.");
  delay(1000);
}
