#include <Arduino.h>
#include "driver/mcpwm.h"
#include "CANHandler.h"   // ★ Make sure CANHandler.h/.cpp are present
#include "Pins.h"         // ★ (If you rely on anything from Pins.h, otherwise you can remove this)

// ----------------------
// Pin Assignments (if not already in Pins.h)
// ----------------------
#define EN_PIN             6
#define PH_PIN             7
#define ENC_A              17
#define ENC_B              18
#define ENC_IDX            8
#define LIMIT_SWITCH_PIN   47  // Limit switch is HIGH when pressed

// ----------------------
// MCPWM Configuration
// ----------------------
#define MCPWM_UNIT   MCPWM_UNIT_0
#define MCPWM_TIMER  MCPWM_TIMER_0
#define MCPWM_GEN    MCPWM_GEN_A

// ----------------------
// Motion Parameters
// ----------------------
#define MAX_DUTY    75
#define MIN_DUTY    10
#define ACCEL_STEP  1
#define LOOP_DELAY  20
#define MIN_POS     10000    // New minimum position (example)

// ----------------------
// State Variables (all of these were in your “original”)
// ----------------------
extern volatile long encoderCount;
volatile long encoderCount = 0;
long homePos            = 0;
long maxPos             = 11000000;
bool autoMove           = false;
bool movingForward      = true;
bool isForward          = true;

int  targetDuty         = 0;
int  currentDuty        = 0;
int  motorState         = 0;  // 0 = stopped, 1 = moving

unsigned long lastPrintTime      = 0;
bool          prevLimitSwitchState = LOW;

bool homing                = false;
bool goToMinAfterHoming    = false;
long moveTarget            = -1;

// ----------------------
// Encoder Interrupt Routine
// ----------------------
void IRAM_ATTR encoderISR() {
  // Increment/decrement based on current direction
  encoderCount += (isForward ? 1 : -1);
}

// ----------------------
// Helper: move to a given encoder count
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
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);

  // Configure motor direction pin and limit switch
  pinMode(PH_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Configure encoder pins & attach interrupt on A‐channel
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_IDX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  // Initialize MCPWM for driving the motor
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency    = 5000;
  pwm_config.cmpr_a       = 0.0;
  pwm_config.cmpr_b       = 0.0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);

  // ★ Initialize CAN so that loopCAN() can start receiving:
  initCAN();

  Serial.println("VBD Controller Started.");
  Serial.println("Serial Commands:");
  Serial.println("  0 = Home & then go to max");
  Serial.println("  1 = Go to max");
  Serial.println("  2 = Go to mid");
  Serial.println("  3 = Go to min");
  Serial.println("  4 = Stop all");
  Serial.println("  5 = Home & then go to min");
  Serial.println("  a = Set Home (encoder=0)");
  Serial.println("  b = Set Max (current encoder → maxPos)");
  Serial.println("  c = Start Auto cycle");
  Serial.println("  x = Stop Auto cycle");
}

// ----------------------
// Main Loop
// ----------------------
void loop() {
  //
  // =========== (A) Poll CAN bus ===========
  //
  loopCAN();  // ★ Must be called every iteration for CAN to work

  if (hasNewCommand()) {
    uint8_t cmd         = getLastCommand();
    float   posFromCAN  = getTargetPosition();  // Only valid if DLC ≥ 3

    switch (cmd) {
      case 1:  // VBD,IN
        Serial.print("CAN → Move to MIN_POS: ");
        Serial.println(MIN_POS);
        moveToPosition(MIN_POS);
        break;

      case 2:  // VBD,MID
        Serial.println("CAN → Move to midpoint.");
        moveToPosition(maxPos / 2);
        break;

      case 3:  // VBD,OUT
        Serial.println("CAN → Move to maxPos.");
        moveToPosition(maxPos);
        break;

      case 4:  // VBD,ZERO
        Serial.println("CAN → Homing (limit switch), then go to MIN_POS");
        homing = true;
        goToMinAfterHoming = true;     // ← now we’ll go TO MIN_POS afterwards
        motorState = 1;
        targetDuty = MAX_DUTY;
        digitalWrite(PH_PIN, LOW);     // Move inward toward limit switch
        isForward = false;
        break;
      
      case 5:  // STOP
        Serial.println("CAN → STOP motor immediately");
        // cancel any motion or homing
        autoMove         = false;
        homing           = false;
        goToMinAfterHoming = false;
        // zero out drive
        motorState       = 0;
        targetDuty       = 0;
        moveTarget       = -1;
        break;

      default:
        Serial.printf("CAN → Unrecognized command: %u\n", cmd);
        break;
    }

  }

  //
  // =========== (B) Serial‐terminal command handling ===========
  //
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '0') {
      Serial.println("Serial → Homing initiated (→ max).");
      homing             = true;
      goToMinAfterHoming = false;
      motorState         = 1;
      targetDuty         = MAX_DUTY;
      digitalWrite(PH_PIN, LOW);
      isForward          = false;

    } else if (input == '1') {
      Serial.println("Serial → Move to maxPos.");
      moveToPosition(maxPos);

    } else if (input == '2') {
      Serial.println("Serial → Move to midpoint.");
      moveToPosition(maxPos / 2);

    } else if (input == '3') {
      Serial.print("Serial → Move to MIN_POS: ");
      Serial.println(MIN_POS);
      moveToPosition(MIN_POS);

    } else if (input == '4') {
      Serial.println("Serial → Stop all motion.");
      autoMove    = false;
      motorState  = 0;
      targetDuty  = 0;
      moveTarget  = -1;

    } else if (input == '5') {
      Serial.println("Serial → Homing initiated (→ min).");
      homing             = true;
      goToMinAfterHoming = true;
      motorState         = 1;
      targetDuty         = MAX_DUTY;
      digitalWrite(PH_PIN, LOW);
      isForward          = false;

    } else if (input == 'a') {
      noInterrupts();
      encoderCount = 0;
      interrupts();
      homePos = 0;
      Serial.println("Serial → Set Home: encoder reset to 0");

    } else if (input == 'b') {
      maxPos = encoderCount;
      Serial.print("Serial → Set Max at encoder = ");
      Serial.println(maxPos);

    } else if (input == 'c') {
      autoMove      = true;
      movingForward = true;
      motorState    = 1;
      digitalWrite(PH_PIN, HIGH);
      isForward     = true;
      Serial.println("Serial → Auto mode started");

    } else if (input == 'x') {
      autoMove    = false;
      motorState  = 0;
      targetDuty  = 0;
      moveTarget  = -1;
      Serial.println("Serial → Auto mode stopped");
    }
  }

  //
  // =========== (C) Limit‐switch & Homing logic ===========
  //
  bool currentLimitSwitchState = digitalRead(LIMIT_SWITCH_PIN);
  if (currentLimitSwitchState == HIGH && prevLimitSwitchState == LOW) {
    // Stop motor immediately
    motorState  = 0;
    targetDuty  = 0;
    currentDuty = 0;

    // Reset encoder count to zero → that is “home”
    noInterrupts();
    encoderCount = 0;
    interrupts();
    homePos = 0;

    isForward = true;
    digitalWrite(PH_PIN, HIGH);  // Next direction = forward

    Serial.println("Limit switch hit! Motor stopped & home set.");

    if (homing) {
      homing = false;
      if (goToMinAfterHoming) {
        goToMinAfterHoming = false;
        Serial.println("Homing complete. Moving to MIN_POS…");
        moveToPosition(MIN_POS);
      } else {
        Serial.println("Homing complete. Moving to maxPos…");
        moveToPosition(maxPos);
      }
    }
  }
  prevLimitSwitchState = currentLimitSwitchState;

  //
  // =========== (D) When moving to a fixed target, stop if reached ===========
  //
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

  //
  // =========== (E) Auto‐move (oscillate between homePos and maxPos) ===========
  //
  if (autoMove) {
    long pos      = encoderCount;
    long distance = abs(maxPos - homePos);
    long progress = abs(pos - (movingForward ? homePos : maxPos));
    float pct     = (float)progress / distance;

    // Ramp‐up duty for the first 20% of travel
    int dynamicTarget = (pct < 0.2)
      ? map(pct * 100, 0, 20, MIN_DUTY, MAX_DUTY)
      : MAX_DUTY;
    targetDuty = constrain(dynamicTarget, MIN_DUTY, MAX_DUTY);

    // If we’ve reached one end, flip direction
    if (movingForward && encoderCount >= maxPos) {
      digitalWrite(PH_PIN, LOW);   // Reverse
      isForward     = false;
      movingForward = false;
    }
    else if (!movingForward && encoderCount <= homePos) {
      digitalWrite(PH_PIN, HIGH);  // Forward
      isForward     = true;
      movingForward = true;
    }

    motorState = 1;
  }

  //
  // =========== (F) PWM‐duty smoothing & MCPWM output ===========
  //
  if (currentDuty < targetDuty) {
    currentDuty += ACCEL_STEP;
    if (currentDuty > targetDuty) currentDuty = targetDuty;
  }
  else if (currentDuty > targetDuty) {
    currentDuty -= ACCEL_STEP;
    if (currentDuty < targetDuty) currentDuty = targetDuty;
  }

  mcpwm_set_duty(
    MCPWM_UNIT,
    MCPWM_TIMER,
    MCPWM_GEN,
    (motorState == 0) ? 0 : currentDuty
  );

  delay(LOOP_DELAY);
}
