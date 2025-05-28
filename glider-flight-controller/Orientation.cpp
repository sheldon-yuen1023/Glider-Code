// Orientation.cpp

#include "Orientation.h"
#include "KalmanOrientation.h"

#define ORIENTATION_DISABLED true  // ✅ Set to true to disable IMU access (no I2C transactions)

#if !ORIENTATION_DISABLED
#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;
KalmanOrientation kalman;
#endif

// Orientation state
unsigned long lastOrientationUpdate = 0;
unsigned long lastPrintTime = 0;
const unsigned long updateInterval = 5000;      // in microseconds (200 Hz)
const unsigned long printInterval = 1000000;    // in microseconds (1 Hz)

void initOrientation() {
#if ORIENTATION_DISABLED
  Serial.println("Orientation system disabled.");
  return;
#else
  Wire.begin();
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) { delay(10); }
  }
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }

  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  kalman.begin();
#endif
}

void getOrientation(float& pitch, float& roll, float& yaw) {
#if ORIENTATION_DISABLED
  pitch = roll = yaw = 0.0f;
  return;
#else
  sensors_event_t accel, gyro, mag;
  lsm6ds33.getEvent(&accel, &gyro, NULL, NULL);
  lis3mdl.getEvent(&mag);

  kalman.update(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  pitch = kalman.getPitch();
  roll  = kalman.getRoll();
  yaw   = kalman.getYaw();
#endif
}
