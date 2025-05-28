#include "Telemetry.h"
#include "CANHandler.h"
#include "Orientation.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "Pins.h"

extern HardwareSerial RS485;  // <-- Let main .ino own the UART object

// Dummy placeholders (replace with real data when ready)
int currentSystemStateCode = 3;
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;
float pressureReading = 98.6;
float sonarDistance = 3.4;
float verticalVelocity = -0.12;
float horizontalVelocity = 0.75;
bool leak1 = false;
bool leak2 = false;
bool leak3 = true;
float VBD1_position = 42.1;
float VBD2_position = 38.6;
float Pitch_position = 12.5;
float Roll_position = 3.7;

void TelemetryTask(void* param) {
  Serial.println("[TASK] Starting TelemetryTask");
  vTaskDelay(pdMS_TO_TICKS(500));

  while (true) {
    StaticJsonDocument<512> doc;

    float pitch, roll, yaw;
    getOrientation(pitch, roll, yaw);  // get thread-safe filtered data

    JsonObject vehicle = doc.createNestedObject("vehicle");
    vehicle["pitch"] = pitch;
    vehicle["roll"] = roll;
    vehicle["yaw"] = yaw;

    // Send over USB for debug
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // Send over RS485
    char buffer[256];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));
    RS485.write((const uint8_t*)buffer, len);
    RS485.write('\n');  // Optional line break for delimiting packets

    vTaskDelay(pdMS_TO_TICKS(1000));  // 1 Hz telemetry
  }
}


void startTelemetryTask() {
  xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 8192, NULL, 1, NULL, 1);
}
