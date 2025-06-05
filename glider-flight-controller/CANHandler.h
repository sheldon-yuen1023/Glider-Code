#pragma once

#include <ArduinoJson.h>
#include <Arduino.h>

void initCAN();                 // Call once in setup
void startCANReceiveTask();     // Starts background CAN reader
void getLatestBMS(int index, JsonObject& obj);  // For JSON module
bool getLatestVBD(int index, JsonObject& obj);
void sendVBDCommand(uint8_t vbd_id, uint8_t command);
