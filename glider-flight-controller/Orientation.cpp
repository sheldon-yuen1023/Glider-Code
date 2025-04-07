#include "Orientation.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_Sensor.h>

LSM6 imu;
LIS3MDL mag;
Adafruit_Madgwick filter;

unsigned long lastUpdate = 0;

// === MAGNETOMETER OFFSETS ===
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;
// =============================

void initOrientation() {
  delay(1000);
  Wire.begin(6, 7);
  Wire.setClock(800000);

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

  filter.begin(FILTER_UPDATE_RATE_HZ);
  filter.setBeta(0.3);
  lastUpdate = micros();

  Serial.println("IMU Fusion Ready (9DOF with Madgwick)");
}

void getOrientation(float& pitch, float& roll, float& yaw) {
  imu.read();
  mag.read();

  unsigned long now = micros();
  float dt = (now - lastUpdate) / 10000000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / FILTER_UPDATE_RATE_HZ;
  lastUpdate = now;

  float ax = imu.a.x / 1000.0f;
  float ay = imu.a.y / 1000.0f;
  float az = imu.a.z / 1000.0f;

  float gx = imu.g.x * 0.001f * PI / 180.0f;
  float gy = imu.g.y * 0.001f * PI / 180.0f;
  float gz = imu.g.z * 0.001f * PI / 180.0f;

  float mx = (mag.m.x * 0.1f) - magOffsetX;
  float my = (mag.m.y * 0.1f) - magOffsetY;
  float mz = (mag.m.z * 0.1f) - magOffsetZ;

  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  pitch = filter.getPitch();
  roll = filter.getRoll();
  yaw = filter.getYaw();
}
