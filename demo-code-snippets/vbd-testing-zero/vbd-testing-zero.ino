#include <Arduino.h>
#include "driver/mcpwm.h"

// ----------------------
// Pin Assignments
// ----------------------
#define EN_PIN 6
#define PH_PIN 7
#define ENC_A 17
#define ENC_B 18
#define ENC_IDX 8
#define LIMIT_SWITCH_PIN 47  // Limit switch is HIGH when pressed

// ----------------------
// MCPWM Configuration
// ----------------------
#define MCPWM_UNIT MCPWM_UNIT_0
#define MCPWM_TIMER MCPWM_TIMER_0
#define MCPWM_GEN MCPWM_GEN_A

// ----------------------
// Motion Parameters
// ----------------------
#define MAX_DUTY 40
#define MIN_DUTY 10
#define ACCEL_STEP 1
#define LOOP_DELAY 20
#define MIN_POS 50000  // New minimum position

// ----------------------
// State Variables
// ----------------------
volatile long encoderCount = 0;
long homePos = 0;
long maxPos = 20000000;
bool autoMove = false;
bool movingForward = true;
bool isForward = true;

int targetDuty = 0;
int currentDuty = 0;
int motorState = 0; // 0 = stop, 1 = moving

unsigned long lastPrintTime = 0;
bool prevLimitSwitchState = LOW;

bool homing = false;
bool goToMinAfterHoming = false;
long moveTarget = -1;

// ----------------------
// Encoder Interrupt
// ----------------------
void IRAM_ATTR encoderISR() {
  encoderCount += (isForward ? 1 : -1);
}

// ----------------------
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);

  pinMode(PH_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_IDX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;
  pwm_config.cmpr_a = 0.0;
  pwm_config.cmpr_b = 0.0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);

  Serial.println("Commands:");
  Serial.println("0 = Home and go to max | 1 = Go to max | 2 = Go to mid | 3 = Go to min | 4 = Stop all | 5 = Home and go to min");
  Serial.println("a = Set Home | b = Set Max | c = Start Auto | x = Stop Auto");
}

// ----------------------
// Helper Function
// ----------------------
void moveToPosition(long target) {
  moveTarget = target;
  if (encoderCount < target) {
    digitalWrite(PH_PIN, HIGH);
    isForward = true;
  } else {
    digitalWrite(PH_PIN, LOW);
    isForward = false;
  }
  motorState = 1;
  targetDuty = MAX_DUTY;
}

// ----------------------
// Main Loop
// ----------------------
void loop() {
  // ---- Serial Input ----
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '0') {
      Serial.println("Homing initiated...");
      homing = true;
      goToMinAfterHoming = false;
      motorState = 1;
      targetDuty = MAX_DUTY;
      digitalWrite(PH_PIN, LOW);
      isForward = false;
    } else if (input == '1') {
      Serial.println("Moving to maxPos");
      moveToPosition(maxPos);
    } else if (input == '2') {
      Serial.println("Moving to midpoint");
      moveToPosition(maxPos / 2);
    } else if (input == '3') {
      Serial.print("Moving to minimum position: ");
      Serial.println(MIN_POS);
      moveToPosition(MIN_POS);
    } else if (input == '4') {
      Serial.println("Stopping all motion.");
      autoMove = false;
      motorState = 0;
      targetDuty = 0;
      moveTarget = -1;
    } else if (input == '5') {
      Serial.println("Homing initiated, will go to MIN_POS after...");
      homing = true;
      goToMinAfterHoming = true;
      motorState = 1;
      targetDuty = MAX_DUTY;
      digitalWrite(PH_PIN, LOW); // Reverse
      isForward = false;
    } else if (input == 'a') {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      homePos = 0;
      Serial.println("Set Home: Encoder reset to 0");
    } else if (input == 'b') {
      maxPos = encoderCount;
      Serial.print("Set Max at: ");
      Serial.println(maxPos);
    } else if (input == 'c') {
      autoMove = true;
      movingForward = true;
      motorState = 1;
      digitalWrite(PH_PIN, HIGH);
      isForward = true;
      Serial.println("Auto mode started");
    } else if (input == 'x') {
      autoMove = false;
      motorState = 0;
      targetDuty = 0;
      moveTarget = -1;
      Serial.println("Auto mode stopped");
    }
  }

  // ---- Limit Switch Handling ----
  bool currentLimitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
  if (currentLimitSwitchState == HIGH && prevLimitSwitchState == LOW) {
    motorState = 0;
    targetDuty = 0;
    currentDuty = 0;

    noInterrupts();
    encoderCount = 0;
    interrupts();

    homePos = 0;
    isForward = true;
    digitalWrite(PH_PIN, HIGH);  // Set direction to forward

    Serial.println("Limit switch hit! Motor stopped and home set.");

    if (homing) {
      homing = false;
      if (goToMinAfterHoming) {
        goToMinAfterHoming = false;
        Serial.println("Homing complete. Moving to MIN_POS...");
        moveToPosition(MIN_POS);
      } else {
        Serial.println("Homing complete. Moving to maxPos...");
        moveToPosition(maxPos);
      }
    }
  }
  prevLimitSwitchState = currentLimitSwitchState;

  // ---- Targeted Position Move ----
  if (!homing && moveTarget >= 0) {
    bool reached = (isForward && encoderCount >= moveTarget) ||
                   (!isForward && encoderCount <= moveTarget);
    if (reached) {
      motorState = 0;
      targetDuty = 0;
      moveTarget = -1;
      Serial.println("Target position reached.");
    }
  }

  // ---- Auto Move Logic ----
  if (autoMove) {
    long pos = encoderCount;
    long distance = abs(maxPos - homePos);
    long progress = abs(pos - (movingForward ? homePos : maxPos));
    float pct = (float)progress / distance;

    int dynamicTarget = (pct < 0.2) ? map(pct * 100, 0, 20, MIN_DUTY, MAX_DUTY) : MAX_DUTY;
    targetDuty = constrain(dynamicTarget, MIN_DUTY, MAX_DUTY);

    if (movingForward && encoderCount >= maxPos) {
      digitalWrite(PH_PIN, LOW);  // Reverse
      isForward = false;
      movingForward = false;
    } else if (!movingForward && encoderCount <= homePos) {
      digitalWrite(PH_PIN, HIGH); // Forward
      isForward = true;
      movingForward = true;
    }

    motorState = 1;
  }

  // ---- PWM Duty Smoothing ----
  if (currentDuty < targetDuty) {
    currentDuty += ACCEL_STEP;
    if (currentDuty > targetDuty) currentDuty = targetDuty;
  } else if (currentDuty > targetDuty) {
    currentDuty -= ACCEL_STEP;
    if (currentDuty < targetDuty) currentDuty = targetDuty;
  }

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, (motorState == 0) ? 0 : currentDuty);

  // ---- Live Encoder Display ----
  if (millis() - lastPrintTime >= 200) {
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
    lastPrintTime = millis();
  }

  delay(LOOP_DELAY);
}

