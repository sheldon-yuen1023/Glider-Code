#pragma once

#include <ArduinoJson.h>

#include <Arduino.h>

void initCAN();                 // Call once in setup
void startCANReceiveTask();     // Starts background CAN reader
void getLatestBMS(int index, JsonObject& obj);  // For JSON module
void getLatestVBD(int index, JsonObject& obj);
