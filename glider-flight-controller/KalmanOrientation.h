#ifndef KALMAN_ORIENTATION_H
#define KALMAN_ORIENTATION_H

#include <Arduino.h>

// This class estimates the orientation of the device (pitch, roll, yaw)
// by combining accelerometer, gyroscope, and magnetometer data.
// It uses a simple Kalman-style complementary filter for fusion.
class KalmanOrientation {
public:
  // Constructor — sets all angles to zero
  KalmanOrientation();

  // Initializes the filter
  // beta controls how quickly the filter responds:
  // - higher beta = faster response but noisier
  // - lower beta = smoother but slower to adapt
  void begin(float beta = 0.02);

  // Main function to update orientation using the latest IMU readings
  // ax, ay, az — accelerometer in g
  // gx, gy, gz — gyroscope in rad/s
  // mx, my, mz — magnetometer in microtesla (µT)
  // dt         — time since last update in seconds
  void update(float ax, float ay, float az,
              float gx, float gy, float gz,
              float mx, float my, float mz,
              float dt);

  // Get current estimated pitch angle (in degrees)
  float getPitch() const;

  // Get current estimated roll angle (in degrees)
  float getRoll() const;

  // Get current estimated yaw/heading angle (in degrees, 0–360)
  float getYaw() const;

private:
  // Internal orientation state (in radians)
  float pitch;
  float roll;
  float yaw;

  // Filter gain (responsiveness tuning)
  float beta;

  // Estimates pitch and roll using accelerometer data
  void updateAnglesFromAccel(float ax, float ay, float az, float& pitchAcc, float& rollAcc);

  // Computes yaw using magnetometer and tilt correction
  float computeTiltCompensatedYaw(float mx, float my, float mz, float pitch, float roll);
};

#endif
