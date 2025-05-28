#include "Orientation.h"
#include "KalmanOrientation.h"
#include <Wire.h>
#include <LSM6.h>       // Accelerometer + Gyroscope
#include <LIS3MDL.h>     // Magnetometer
#include "Pins.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Sensor instances
LSM6 imu;
LIS3MDL mag;
KalmanOrientation kalman;

// Shared filtered orientation values
volatile float sharedPitch = 0;
volatile float sharedRoll = 0;
volatile float sharedYaw = 0;

// Thread-safe lock for shared values
SemaphoreHandle_t orientationMutex;

// Optional magnetometer offset correction
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;

// FreeRTOS task that handles IMU read + Kalman filtering (runs on Core 0)
void orientationTask(void* parameter) {
  unsigned long lastUpdate = micros();

  while (true) {
    imu.read();
    mag.read();

    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
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

    kalman.update(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

    // Update shared orientation values safely
    if (xSemaphoreTake(orientationMutex, 0) == pdTRUE) {
      sharedPitch = kalman.getPitch();
      sharedRoll  = kalman.getRoll();
      sharedYaw   = kalman.getYaw();
      xSemaphoreGive(orientationMutex);
    }

    // Delay to maintain update frequency
    delayMicroseconds(1000000 / FILTER_UPDATE_RATE_HZ);
  }
}

// Call this in setup() to initialize sensors and start the filter task
void initOrientation() {
  delay(1000);
  Wire.begin(IMU_SDA, IMU_SCL);          // Replace with your board's SDA, SCL pins
  Wire.setClock(800000);     // Fast I2C for high IMU polling rate

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

  kalman.begin(0.05); // Filter responsiveness: 0.01–0.1 typical range
  orientationMutex = xSemaphoreCreateMutex();

  // Launch IMU fusion task on Core 0
  xTaskCreatePinnedToCore(
    orientationTask,      // Task function
    "OrientationTask",    // Name
    4096,                 // Stack size
    NULL,                 // Params
    1,                    // Priority
    NULL,                 // Task handle
    0                     // Run on Core 0
  );

  Serial.println("Orientation task started on Core 0.");
}

// Thread-safe accessor for current orientation
void getOrientation(float& pitch, float& roll, float& yaw) {
  if (xSemaphoreTake(orientationMutex, 0) == pdTRUE) {
    pitch = sharedPitch;
    roll  = sharedRoll;
    yaw   = sharedYaw;
    xSemaphoreGive(orientationMutex);
  }
}
  