#include "KalmanOrientation.h"
#include <math.h>

KalmanOrientation::KalmanOrientation()
  : pitch(0), roll(0), yaw(0), beta(0.02f) {}

void KalmanOrientation::begin(float betaVal) {
  beta = betaVal;
}

void KalmanOrientation::update(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               float mx, float my, float mz,
                               float dt) {
  // Integrate gyroscope to estimate orientation
  pitch += gx * dt;
  roll  += gy * dt;
  yaw   += gz * dt;

  // Calculate pitch and roll from accelerometer
  float pitchAcc, rollAcc;
  updateAnglesFromAccel(ax, ay, az, pitchAcc, rollAcc);

  // Complementary fusion
  pitch = (1.0f - beta) * pitch + beta * pitchAcc;
  roll  = (1.0f - beta) * roll + beta * rollAcc;

  // Tilt-compensated yaw from magnetometer
  float yawMag = computeTiltCompensatedYaw(mx, my, mz, pitch, roll);
  yaw = (1.0f - beta) * yaw + beta * yawMag;
}

float KalmanOrientation::getPitch() const {
  return pitch * 180.0f / PI;
}

float KalmanOrientation::getRoll() const {
  return roll * 180.0f / PI;
}

float KalmanOrientation::getYaw() const {
  float deg = yaw * 180.0f / PI;
  return (deg < 0) ? (deg + 360.0f) : deg;
}

void KalmanOrientation::updateAnglesFromAccel(float ax, float ay, float az, float& pitchAcc, float& rollAcc) {
  rollAcc  = atan2(ay, sqrt(ax * ax + az * az)); 
  pitchAcc = atan2(-ax, az);                      

}

float KalmanOrientation::computeTiltCompensatedYaw(float mx, float my, float mz, float pitch, float roll) {
  float sinPitch = sin(pitch);
  float cosPitch = cos(pitch);
  float sinRoll = sin(roll);
  float cosRoll = cos(roll);

  float Xh = mx * cosPitch + mz * sinPitch;
  float Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

  return atan2(-Yh, Xh);
}
