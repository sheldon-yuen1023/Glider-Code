#include "Orientation.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <MadgwickAHRS.h>

LSM6 imu;
LIS3MDL mag;
Madgwick filter;

unsigned long lastUpdate = 0;

// Optional: magnetometer calibration offsets (replace with your own if needed)
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;

void initOrientation() {
  delay(1000);
  Wire.begin(6, 7);           // Replace with your SDA/SCL pins
  Wire.setClock(800000);      // Optional: faster I2C

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

  filter.begin(FILTER_UPDATE_RATE_HZ);  // only needed for internal beta
  lastUpdate = micros();

  Serial.println("IMU Fusion Ready (9DOF with dt-based Madgwick)");
}

void getOrientation(float& pitch, float& roll, float& yaw) {
  imu.read();
  mag.read();

  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / FILTER_UPDATE_RATE_HZ;
  lastUpdate = now;

  // Accel in g
  float ax = imu.a.x / 1000.0f;
  float ay = imu.a.y / 1000.0f;
  float az = imu.a.z / 1000.0f;

  // Gyro in rad/s
  float gx = imu.g.x * 0.001f * PI / 180.0f;
  float gy = imu.g.y * 0.001f * PI / 180.0f;
  float gz = imu.g.z * 0.001f * PI / 180.0f;

  // Mag in µT and apply calibration offsets
  float mx = (mag.m.x * 0.1f) - magOffsetX;
  float my = (mag.m.y * 0.1f) - magOffsetY;
  float mz = (mag.m.z * 0.1f) - magOffsetZ;

  // Normalize magnetometer vector (optional but helps with noisy readings)
  float norm = sqrt(mx*mx + my*my + mz*mz);
  if (norm > 0.01f) {
    mx /= norm;
    my /= norm;
    mz /= norm;
  }

  // Full 9DOF update with dt
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Euler angles in degrees
  pitch = filter.getPitch();
  roll  = filter.getRoll();
  yaw   = filter.getYaw();
}
