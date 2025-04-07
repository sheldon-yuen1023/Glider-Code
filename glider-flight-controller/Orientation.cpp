#include "Orientation.h"
#include <Wire.h>
#include <LSM6.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_Sensor.h>

LSM6 imu;
Adafruit_Madgwick filter;

unsigned long lastUpdate = 0;

void initOrientation() {
  delay(1000);
  Wire.begin(6, 7);           // SDA, SCL (adjust if needed)
  Wire.setClock(800000);      // Fast I2C

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1);
  }
  imu.enableDefault();

  filter.begin(FILTER_UPDATE_RATE_HZ); // Fixed update rate for internal scaling
  filter.setBeta(0.4);                 // Increase for responsiveness
  lastUpdate = micros();

  Serial.println("IMU Fusion Ready (6DOF, no magnetometer)");
}

void getOrientation(float& pitch, float& roll, float& yaw) {
  imu.read();

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

  // 6DOF update (no magnetometer)
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  pitch = filter.getPitch();
  roll = filter.getRoll();
  yaw = filter.getYaw();  // Will drift over time
}
