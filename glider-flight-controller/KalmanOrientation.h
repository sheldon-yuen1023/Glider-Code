#ifndef KALMAN_ORIENTATION_H
#define KALMAN_ORIENTATION_H

#include <Arduino.h>

class KalmanOrientation {
public:
  KalmanOrientation();

  void begin(float beta = 0.02);
  void update(float ax, float ay, float az,
              float gx, float gy, float gz,
              float mx, float my, float mz,
              float dt);

  float getPitch() const;
  float getRoll() const;
  float getYaw() const;

private:
  float pitch, roll, yaw;
  float beta;

  void updateAnglesFromAccel(float ax, float ay, float az, float& pitchAcc, float& rollAcc);
  float computeTiltCompensatedYaw(float mx, float my, float mz, float pitch, float roll);
};

#endif
