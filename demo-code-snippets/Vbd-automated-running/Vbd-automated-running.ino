#include <Arduino.h>
#include "driver/mcpwm.h"

// ----------------------
// Pin Assignments
// ----------------------
#define EN_PIN 7        // PWM control pin
#define PH_PIN 6        // Direction control pin
#define ENC_A 17        // Encoder Channel A
#define ENC_B 18        // Encoder Channel B
#define ENC_IDX 8       // Encoder Index (not used but reserved)

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
#define ACCEL_STEP 2
#define LOOP_DELAY 50  // Loop speed (ms)

// ----------------------
// Encoder & Motion State
// ----------------------
volatile long encoderCount = 0;
long homePos = 0;
long maxPos = 1000;  // Default max position

bool autoMove = false;
bool movingForward = true;

int targetDuty = 0;
int currentDuty = 0;
int motorState = 0; // 0 = stop, 1 = forward, 2 = reverse

unsigned long lastPrintTime = 0;

// ----------------------
// Encoder Interrupt
// ----------------------
void IRAM_ATTR encoderISR() {
  int b = digitalRead(ENC_B);
  encoderCount += (b == HIGH) ? 1 : -1;
}

// ----------------------
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);

  pinMode(PH_PIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_IDX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  // Initialize MCPWM
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;
  pwm_config.cmpr_a = 0.0;
  pwm_config.cmpr_b = 0.0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);

  Serial.println("Commands:");
  Serial.println("1 = Forward | 2 = Reverse | 3 = Stop");
  Serial.println("a = Set Home | b = Set Max | c = Start Auto | x = Stop Auto");
}

// ----------------------
// Main Loop
// ----------------------
void loop() {
  // ---- Handle Serial Commands ----
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '1') {
      motorState = 1;
      digitalWrite(PH_PIN, LOW); // Forward
      targetDuty = MAX_DUTY;
      autoMove = false;
      Serial.println("Manual: Forward");
    } else if (input == '2') {
      motorState = 2;
      digitalWrite(PH_PIN, HIGH); // Reverse
      targetDuty = MAX_DUTY;
      autoMove = false;
      Serial.println("Manual: Reverse");
    } else if (input == '3') {
      motorState = 0;
      targetDuty = 0;
      autoMove = false;
      Serial.println("Manual: Stop");
    } else if (input == 'a') {
      homePos = encoderCount;
      Serial.print("Set home at: ");
      Serial.println(homePos);
    } else if (input == 'b') {
      maxPos = encoderCount;
      Serial.print("Set max at: ");
      Serial.println(maxPos);
    } else if (input == 'c') {
      autoMove = true;
      motorState = 1;
      movingForward = true;
      digitalWrite(PH_PIN, LOW);
      Serial.println("Auto mode started");
    } else if (input == 'x') {
      autoMove = false;
      motorState = 0;
      targetDuty = 0;
      Serial.println("Auto mode stopped");
    }
  }

  // ---- Auto Movement Logic ----
  if (autoMove) {
    if (movingForward && encoderCount >= maxPos) {
      digitalWrite(PH_PIN, HIGH);
      movingForward = false;
    } else if (!movingForward && encoderCount <= homePos) {
      digitalWrite(PH_PIN, LOW);
      movingForward = true;
    }
    motorState = 1;
    targetDuty = MAX_DUTY;
  }

  // ---- PWM Duty Control ----
  if (currentDuty < targetDuty) {
    currentDuty += ACCEL_STEP;
    if (currentDuty > targetDuty) currentDuty = targetDuty;
  } else if (currentDuty > targetDuty) {
    currentDuty -= ACCEL_STEP;
    if (currentDuty < targetDuty) currentDuty = targetDuty;
  }

  mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, (motorState == 0) ? 0 : currentDuty);

  // ---- Real-Time Encoder Display ----
  if (millis() - lastPrintTime >= 200) {
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
    lastPrintTime = millis();
  }

  delay(LOOP_DELAY);
}
