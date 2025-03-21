#include "Orientation.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_Sensor.h>

LSM6 imu;
LIS3MDL mag;
Adafruit_Madgwick filter;

float smoothedPitch = 0.0;
float smoothedRoll = 0.0;
float alpha = 0.6;
unsigned long lastUpdate = 0;

// === MAGNETOMETER OFFSETS ===
float magOffsetX = -89.55;
float magOffsetY = -95.90;
float magOffsetZ = 117.60;
// =============================

void initOrientation() {
  delay(1000);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1);
  }
  imu.enableDefault();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  filter.begin(100); // 100 Hz
  filter.setBeta(0.1);
  lastUpdate = millis();

  Serial.println("IMU Fusion Ready (Gyro + Accel) + Compass Heading");
}

void getOrientation(float& pitch, float& roll, float& yaw) {
  imu.read();
  mag.read();

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  float ax = imu.a.x * 9.81 / 1000.0;
  float ay = imu.a.y * 9.81 / 1000.0;
  float az = imu.a.z * 9.81 / 1000.0;

  float gx = imu.g.x * 0.001 * PI / 180.0;
  float gy = imu.g.y * 0.001 * PI / 180.0;
  float gz = imu.g.z * 0.001 * PI / 180.0;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float rawPitch = filter.getPitch();
  float rawRoll  = filter.getRoll();

  smoothedPitch = alpha * smoothedPitch + (1 - alpha) * rawPitch;
  smoothedRoll  = alpha * smoothedRoll + (1 - alpha) * rawRoll;

  // Apply magnetometer offset correction
  float mx = (mag.m.x * 0.1) - magOffsetX;
  float my = (mag.m.y * 0.1) - magOffsetY;
  float mz = (mag.m.z * 0.1) - magOffsetZ;

  float pitchRad = smoothedPitch * PI / 180.0;
  float rollRad  = smoothedRoll  * PI / 180.0;

  float mx_comp = mx * cos(pitchRad) + mz * sin(pitchRad);
  float my_comp = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);

  float heading = atan2(-my_comp, mx_comp) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  pitch = smoothedPitch;
  roll = smoothedRoll;
  yaw = heading;
}
