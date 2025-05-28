#pragma once

#include <HardwareSerial.h>
#include "Pins.h"

extern HardwareSerial RS485;

void startTelemetryTask();  // Call in setu