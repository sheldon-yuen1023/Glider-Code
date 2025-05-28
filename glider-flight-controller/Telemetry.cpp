#include "Telemetry.h"
#include "CANHandler.h"
#include "Orientation.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "Pins.h"

extern HardwareSerial RS485;

// Dummy placeholders (replace with real data when ready)
int currentSystemStateCode = 3;
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
  vTaskDelay(pdMS_TO_TICKS(500));  // Give other systems time to initialize

  while (true) {
    StaticJsonDocument<1024> doc;
    doc["timestamp"] = millis();

    // BMS section
    JsonObject bms = doc.createNestedObject("bms");
    for (int i = 0; i < 5; i++) {
      String key = "bms" + String(i + 1);
      JsonObject entry = bms.createNestedObject(key);
      getLatestBMS(i, entry);  // From CANHandler
    }

    // Orientation (get thread-safe filtered values)
    float pitch, roll, yaw;
    getOrientation(pitch, roll, yaw);

    // Vehicle state
    JsonObject vehicle = doc.createNestedObject("vehicle");
    vehicle["stateCode"] = currentSystemStateCode;
    vehicle["pitch"] = pitch;
    vehicle["roll"] = roll;
    vehicle["yaw"] = yaw;

    // Sensors
    JsonObject sensors = doc.createNestedObject("sensors");
    sensors["pressure"] = pressureReading;
    sensors["distanceToBottom"] = sonarDistance;
    sensors["verticalVelocity"] = verticalVelocity;
    sensors["horizontalVelocity"] = horizontalVelocity;

    JsonObject leak = sensors.createNestedObject("leakSensors");
    leak["sensor1"] = leak1;
    leak["sensor2"] = leak2;
    leak["sensor3"] = leak3;

    // Actuators
    JsonObject actuators = doc.createNestedObject("actuators");
    actuators["vbd1Position"] = VBD1_position;
    actuators["vbd2Position"] = VBD2_position;
    actuators["pitchPosition"] = Pitch_position;
    actuators["rollPosition"] = Roll_position;

    // Send over RS485 and USB for debug
    serializeJson(doc, RS485);
    RS485.write('\n');  // Line break to delimit packets

    serializeJsonPretty(doc, Serial);
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(1000));  // 1 Hz telemetry
  }
}

void startTelemetryTask() {
  xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 8192, NULL, 1, NULL, 1);
}
