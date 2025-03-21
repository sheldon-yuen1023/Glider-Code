#include "Orientation.h"

void setup() {
  Serial.begin(115200);
  initOrientation();
}

void loop() {
  float pitch, roll, yaw;
  getOrientation(pitch, roll, yaw);

  Serial.print("Pitch: "); Serial.print(pitch, 2);
  Serial.print("°, Roll: "); Serial.print(roll, 2);
  Serial.print("°, Heading: "); Serial.print(yaw, 2);
  Serial.println("°");

  delay(20);
}
