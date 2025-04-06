#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDL mag;

float magX_min = 10000, magX_max = -10000;
float magY_min = 10000, magY_max = -10000;
float magZ_min = 10000, magZ_max = -10000;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  if (!mag.init()) {
    Serial.println("Failed to detect LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  Serial.println("Magnetometer Calibration Tool");
  Serial.println("→ Slowly rotate the sensor in all directions for 30–60 seconds.");
  Serial.println("→ Try to make figure-8 motions or rotate on all axes.");
  Serial.println("→ Offsets will be calculated automatically.\n");

  startTime = millis();
}

void loop() {
  mag.read();

  float mx = mag.m.x * 0.1;
  float my = mag.m.y * 0.1;
  float mz = mag.m.z * 0.1;

  // Track min/max
  if (mx < magX_min) magX_min = mx;
  if (mx > magX_max) magX_max = mx;
  if (my < magY_min) magY_min = my;
  if (my > magY_max) magY_max = my;
  if (mz < magZ_min) magZ_min = mz;
  if (mz > magZ_max) magZ_max = mz;

  // Show live data
  Serial.print("X: "); Serial.print(mx);
  Serial.print(" Y: "); Serial.print(my);
  Serial.print(" Z: "); Serial.println(mz);

  // Show offsets after 60 seconds
  if (millis() - startTime > 60000) {
    float offsetX = (magX_max + magX_min) / 2.0;
    float offsetY = (magY_max + magY_min) / 2.0;
    float offsetZ = (magZ_max + magZ_min) / 2.0;

    Serial.println("\n=== Calibration Complete ===");
    Serial.print("Offset X: "); Serial.println(offsetX, 2);
    Serial.print("Offset Y: "); Serial.println(offsetY, 2);
    Serial.print("Offset Z: "); Serial.println(offsetZ, 2);

    Serial.println("\nPaste these values into your main sketch under MAGNETOMETER OFFSETS section.");
    while (1); // Stop here
  }

  delay(200);
}
