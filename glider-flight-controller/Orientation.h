#ifndef ORIENTATION_H
#define ORIENTATION_H

// Defines how often the orientation filter should update (in Hz).
// This value should match the rate you're calling getOrientation().
#define FILTER_UPDATE_RATE_HZ 200

// Call this once in setup() to initialize sensors and the orientation filter.
void initOrientation();

// Call this repeatedly (e.g. every 5 ms) to get the latest pitch, roll, and yaw angles.
// Outputs are in degrees.
void getOrientation(float& pitch, float& roll, float& yaw);

#endif
