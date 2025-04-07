#include "Orientation.h"
#include "KalmanOrientation.h"
#include <Wire.h>
#include <LSM6.h>       // Accelerometer + Gyroscope
#include <LIS3MDL.h>     // Magnetometer

// IMU sensor instances
LSM6 imu;
LIS3MDL mag;

// Orientation filter instance
KalmanOrientation kalman;

// Tracks time between updates (in microseconds)
unsigned long lastUpdate = 0;

// Magnetometer offsets for basic hard-iron correction (calibrate manually if needed)
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;

// Initializes I2C, IMU sensors, and orientation filter
void initOrientation() {
  delay(1000);                // Allow time for sensors to power up
  Wire.begin(6, 7);           // Start I2C using GPIO 6 (SDA) and 7 (SCL)
  Wire.setClock(800000);      // Use 800 kHz I2C for faster communication

  // Initialize accelerometer + gyroscope
  if (!imu.init()) {
    Serial.println("Failed to detect LSM6!");
    while (1); // Stop if sensor is not detected
  }
  imu.enableDefault();

  // Initialize magnetometer
  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL magnetometer!");
    while (1); // Stop if sensor is not detected
  }
  mag.enableDefault();

  // Initialize Kalman-style orientation filter
  kalman.begin(0.05); // beta = 0.05 → balance between responsiveness and stability

  lastUpdate = micros(); // Timestamp for first update

  Serial.println("Kalman-based IMU Orientation (9DOF) ready.");
}

// Reads sensor values, updates orientation estimate, and outputs pitch/roll/yaw
void getOrientation(float& pitch, float& roll, float& yaw) {
  imu.read();  // Get accelerometer and gyroscope data
  mag.read();  // Get magnetometer data

  // Compute time since last update (in seconds)
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  
  // Clamp unreasonable time deltas (e.g., after reset or overflow)
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 1.0f / FILTER_UPDATE_RATE_HZ;
  }
  lastUpdate = now;

  // Convert raw accelerometer values (mg → g)
  float ax = imu.a.x / 1000.0f;
  float ay = imu.a.y / 1000.0f;
  float az = imu.a.z / 1000.0f;

  // Convert raw gyroscope values (mdps → rad/s)
  float gx = imu.g.x * 0.001f * PI / 180.0f;
  float gy = imu.g.y * 0.001f * PI / 180.0f;
  float gz = imu.g.z * 0.001f * PI / 180.0f;

  // Convert raw magnetometer values (mgauss → µT), apply offsets
  float mx = (mag.m.x * 0.1f) - magOffsetX;
  float my = (mag.m.y * 0.1f) - magOffsetY;
  float mz = (mag.m.z * 0.1f) - magOffsetZ;

  // Update orientation estimate using all 9 sensor axes
  kalman.update(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

  // Return angles in degrees
  pitch = kalman.getPitch();
  roll  = kalman.getRoll();
  yaw   = kalman.getYaw();
}
