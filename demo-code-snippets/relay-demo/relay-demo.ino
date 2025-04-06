#define MOSFET_PIN 2  // GPIO0 on XIAO ESP32-C3

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Testing DFRobot Gravity MOSFET Power Controller");
}

void loop() {
  Serial.println("MOSFET ON");
  digitalWrite(MOSFET_PIN, HIGH); // Turn on MOSFET
  delay(3000); // wait 3 seconds

  Serial.println("MOSFET OFF");
  digitalWrite(MOSFET_PIN, LOW);  // Turn off MOSFET
  delay(3000); // wait 3 seconds
}
