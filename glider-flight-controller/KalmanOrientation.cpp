#include "KalmanOrientation.h"
#include <math.h>

// Constructor — initializes orientation states and filter gain
KalmanOrientation::KalmanOrientation()
  : pitch(0), roll(0), yaw(0), beta(0.02f) {}

// Set the filter gain (responsiveness). 
// Higher beta = quicker reaction to sensor input but more noise.
void KalmanOrientation::begin(float betaVal) {
  beta = betaVal;
}

// Main update function. Call this at a fixed rate with the latest sensor values.
// ax, ay, az = acceleration in g
// gx, gy, gz = angular velocity in rad/s
// mx, my, mz = magnetic field in µT
// dt = time since last update in seconds
void KalmanOrientation::update(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               float mx, float my, float mz,
                               float dt) {
  // 1. Predict orientation change using gyroscope integration
  pitch += gx * dt;
  roll  += gy * dt;
  yaw   += gz * dt;

  // 2. Estimate pitch and roll from accelerometer
  float pitchAcc, rollAcc;
  updateAnglesFromAccel(ax, ay, az, pitchAcc, rollAcc);

  // 3. Apply complementary filter to blend gyro and accel
  pitch = (1.0f - beta) * pitch + beta * pitchAcc;
  roll  = (1.0f - beta) * roll + beta * rollAcc;

  // 4. Update yaw using tilt-compensated magnetometer
  float yawMag = computeTiltCompensatedYaw(mx, my, mz, pitch, roll);
  yaw = (1.0f - beta) * yaw + beta * yawMag;
}

// Returns pitch angle in degrees
float KalmanOrientation::getPitch() const {
  return pitch * 180.0f / PI;
}

// Returns roll angle in degrees
float KalmanOrientation::getRoll() const {
  return roll * 180.0f / PI;
}

// Returns yaw angle in degrees, wrapped to [0, 360)
float KalmanOrientation::getYaw() const {
  float deg = yaw * 180.0f / PI;
  return (deg < 0) ? (deg + 360.0f) : deg;
}

// Compute pitch and roll from raw accelerometer data.
// Outputs are in radians.
void KalmanOrientation::updateAnglesFromAccel(float ax, float ay, float az, float& pitchAcc, float& rollAcc) {
  // Roll is rotation around X (side-to-side tilt)
  rollAcc  = atan2(ay, sqrt(ax * ax + az * az));

  // Pitch is rotation around Y (front-to-back tilt)
  pitchAcc = atan2(-ax, az);
}

// Compute tilt-compensated heading (yaw) using magnetometer data.
// Returns yaw in radians.
float KalmanOrientation::computeTiltCompensatedYaw(float mx, float my, float mz, float pitch, float roll) {
  // Apply pitch and roll compensation to the magnetometer data
  float sinPitch = sin(pitch);
  float cosPitch = cos(pitch);
  float sinRoll = sin(roll);
  float cosRoll = cos(roll);

  // Compensate for tilt
  float Xh = mx * cosPitch + mz * sinPitch;
  float Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

  // Calculate yaw (heading) from the compensated magnetic field
  return atan2(-Yh, Xh);
}
