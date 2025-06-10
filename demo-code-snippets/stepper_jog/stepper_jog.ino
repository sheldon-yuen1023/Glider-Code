#include <SPI.h>
#include <HighPowerStepperDriver.h>

// ----------------------
// Motion Parameters
// ----------------------
#define MAX_STEP_RATE_US 1500     // Slower = more torque
#define MIN_STEP_RATE_US 3000     // Stop speed delay
#define ACCEL_STEP       5        // Gentle ramp-up
#define ACCEL_INTERVAL   10       // Change delay every N steps

// ----------------------
// Motor Setup
// ----------------------
const uint8_t CSPin = 10;
HighPowerStepperDriver sd;

int commandState = 0;             // 0 = stop, 1 = forward, 2 = reverse
int currentDelay = MIN_STEP_RATE_US;
int targetDelay  = MIN_STEP_RATE_US;
bool motorActive = false;

unsigned long lastStepTime = 0;
int stepCounter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB Serial to become available
  }
  delay(200);  // give your host a moment
  Serial.println("Type '1' = forward, '2' = reverse, '3' = stop");

  SPI.begin();
  sd.setChipSelectPin(CSPin);
  delay(10);

  // Configure motor driver
  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  sd.setCurrentMilliamps36v4(4000);          // High current for torque
  sd.setStepMode(HPSDStepMode::MicroStep8);        // Full step = max torque
  sd.enableDriver();
  sd.setDirection(0);

  Serial.println("Type '1' = forward, '2' = reverse, '3' = stop");
}

void loop() {
  // --- Handle Serial Input ---
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '1') {
      commandState = 1;
      targetDelay = MAX_STEP_RATE_US;
      sd.setDirection(0);
      motorActive = true;
      Serial.println("Command: Rotate Forward");
    } else if (input == '2') {
      commandState = 2;
      targetDelay = MAX_STEP_RATE_US;
      sd.setDirection(1);
      motorActive = true;
      Serial.println("Command: Rotate Reverse");
    } else if (input == '3') {
      commandState = 0;
      targetDelay = MIN_STEP_RATE_US;
      motorActive = false;
      Serial.println("Command: Stop");
    }
  }

  // --- Stepping Loop ---
  unsigned long now = micros();
  if (motorActive && (now - lastStepTime >= currentDelay)) {
    lastStepTime = now;
    sd.step();
    stepCounter++;

    // Update delay gradually for smooth accel/decel
    if (stepCounter % ACCEL_INTERVAL == 0) {
      if (currentDelay > targetDelay) {
        currentDelay -= ACCEL_STEP;
        if (currentDelay < targetDelay) currentDelay = targetDelay;
      } else if (currentDelay < targetDelay) {
        currentDelay += ACCEL_STEP;
        if (currentDelay > targetDelay) currentDelay = targetDelay;
      }
    }
  }
}