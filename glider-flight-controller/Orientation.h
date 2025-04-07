#ifndef ORIENTATION_H
#define ORIENTATION_H

#define FILTER_UPDATE_RATE_HZ 300

void initOrientation();
void getOrientation(float& pitch, float& roll, float& yaw);

#endif
