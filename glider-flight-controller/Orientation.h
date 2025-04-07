#ifndef ORIENTATION_H
#define ORIENTATION_H

#define FILTER_UPDATE_RATE_HZ 200  // Target filter update rate in Hz

// Call once in setup() to initialize I2C, sensors, and start orientation filter task
void initOrientation();

// Thread-safe function to retrieve the latest filtered pitch, roll, and yaw (in degrees)
void getOrientation(float& pitch, float& roll, float& yaw);

#endif
