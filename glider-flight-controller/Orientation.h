#ifndef ORIENTATION_H
#define ORIENTATION_H

// Set your desired filter update rate in Hz (e.g. 100, 200)
#define FILTER_UPDATE_RATE_HZ 10000 //HERE THE UPDATE RATE CAN BE MODIFIED

void initOrientation();
void getOrientation(float& pitch, float& roll, float& yaw);

#endif
