#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <Arduino.h>

// Function declarations
void initOrientation();
void getOrientation(float& pitch, float& roll, float& yaw);

#endif
