#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <Arduino.h>

enum VBDState : uint8_t {
  VBD_AWAITING_COMMAND = 0,
  VBD_MOVING = 1,
  VBD_HOMING = 2,
  VBD_EMERGENCY = 3
};

void initCAN();
void loopCAN();  // to be called from main loop

// Accessor for received command
bool hasNewCommand();
uint8_t getLastCommand();

// Optional for motor logic to access target, if needed later
float getTargetPosition();

#endif
