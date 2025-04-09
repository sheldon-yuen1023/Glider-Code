#include <Arduino.h>
#include "driver/mcpwm.h"  // Motor Control PWM

// Motor control pins
#define EN_PIN 16             // PWM control pin (connected to EN on driver)
#define PH_PIN 15             // Direction control pin (connected to PH on driver)
#define LIMIT_SWITCH_PIN 18   // Limit switch pin (outputs HIGH when pressed)

// Encoder pins (adjust these as needed)
#define ENC_A_PIN 12          // Encoder Channel A
#define ENC_B_PIN 13          // Encoder Channel B
#define ENC_IDX_PIN 14        // Optional: Encoder Index

volatile long encoderCount = 0;         // Global encoder pulse count
const int countsPerRevolution = 1024;     // Encoder resolution (counts per revolution)

// Motor forward rotation target (in rotations)
// (NOTE: Adjust this to a realistic value; the example uses a large number.)
const float TARGET_ROTATIONS = 200000.0;

// Motion profile parameters (tweak these to suit your hardware)
const float MAX_DUTY_FORWARD = 80.0;    // Maximum duty cycle (forward motion)
const float MAX_DUTY_REVERSE = 30.0;    // Maximum duty cycle (reverse motion)
const float ACCEL_STEP = 5.0;           // PWM duty cycle increment per control loop (approx. every 100ms)
const float DECEL_STEP = 5.0;           // PWM duty cycle decrement per control loop
const float DECELERATION_ROTATIONS = 10.0;  // When within this many rotations of target, start deceleration

// Global variable to track the current PWM duty cycle
float currentDuty = 0.0;

// Define a simple state machine for motor control
enum MotorState {
  REVERSING,      // Motor runs in reverse until the limit switch is pressed
  MOVING_FORWARD, // Motor runs forward until target rotations are reached
  STOPPED         // Motor stops once the target is met
};

MotorState motorState = REVERSING;

// Encoder ISR: counts pulses and determines direction based on Channel B state.
void IRAM_ATTR encoderISR() {
  int bState = digitalRead(ENC_B_PIN);
  if (bState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  // Setup motor control pin; also set the limit switch input (internal pullup)
  pinMode(PH_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  // Start with the motor in reverse mode.
  // In this configuration, assume:
  //   - PH_PIN HIGH selects reverse
  //   - PH_PIN LOW selects forward.
  digitalWrite(PH_PIN, HIGH);

  // Initialize MCPWM for motor speed control with a 5 kHz frequency.
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;        
  // Start at 0% duty cycle for a smooth acceleration.
  pwm_config.cmpr_a = 0.0;
  pwm_config.cmpr_b = 0.0;            
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Setup encoder input pins with internal pull-ups.
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_IDX_PIN, INPUT_PULLUP);  // Optional if you use the index pulse

  // Attach interrupt on the rising edge of the encoder Channel A.
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  // Optional short delay for startup stabilization.
  delay(100);
}

void loop() {
  // --- REVERSING STATE: Accelerate in reverse until the limit switch is pressed ---
  if (motorState == REVERSING) {
    // Ensure the motor is in reverse (PH_PIN HIGH selects reverse).
    digitalWrite(PH_PIN, HIGH);
    
    // Ramp up the reverse speed gradually until the target reverse duty is reached.
    if (currentDuty < MAX_DUTY_REVERSE) {
      currentDuty += ACCEL_STEP;
      if (currentDuty > MAX_DUTY_REVERSE) {
        currentDuty = MAX_DUTY_REVERSE;
      }
    }
    // Set the PWM duty cycle.
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentDuty);
    
    // Check the limit switch: if it reads HIGH, the limit switch is pressed.
    if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
      // When pressed, reset the encoder count atomically.
      noInterrupts();
      encoderCount = 0;
      interrupts();
      
      Serial.println("Limit switch pressed. Encoder reset. Changing direction to FORWARD.");
      
      // Reset the duty cycle so that forward acceleration starts from zero.
      currentDuty = 0.0;
      
      // Change the motor direction to forward (PH_PIN LOW selects forward).
      digitalWrite(PH_PIN, LOW);
      
      // Transition to the MOVING_FORWARD state.
      motorState = MOVING_FORWARD;
    }
  }
  // --- MOVING_FORWARD STATE: Accelerate forward and decelerate as target rotations are neared ---
  else if (motorState == MOVING_FORWARD) {
    // Compute the number of rotations since the encoder was reset.
    float rotations = (float)encoderCount / countsPerRevolution;
    Serial.print("Forward Rotations: ");
    Serial.println(rotations);
    
    // Determine the target PWM duty cycle based on progress.
    float targetDuty;
    float remainingRotations = TARGET_ROTATIONS - rotations;
    
    // Start deceleration when within a specified number of rotations from the target.
    if (remainingRotations <= DECELERATION_ROTATIONS) {
      // Linearly scale the target duty from full speed down to a low value
      targetDuty = (remainingRotations / DECELERATION_ROTATIONS) * MAX_DUTY_FORWARD;
      // Optionally, enforce a minimum duty cycle (if not yet zero) to prevent stalling.
      if (targetDuty < 10.0 && remainingRotations > 0) {
        targetDuty = 10.0;
      }
    } else {
      // Otherwise, aim for maximum forward speed.
      targetDuty = MAX_DUTY_FORWARD;
    }
    
    // Adjust the current duty cycle gradually towards the computed target.
    if (currentDuty < targetDuty) {
      currentDuty += ACCEL_STEP;
      if (currentDuty > targetDuty)
        currentDuty = targetDuty;
    } else if (currentDuty > targetDuty) {
      currentDuty -= DECEL_STEP;
      if (currentDuty < targetDuty)
        currentDuty = targetDuty;
    }
    
    // Apply the updated PWM duty cycle.
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentDuty);
    
    // When the forward rotations reach or exceed the target, stop the motor.
    if (rotations >= TARGET_ROTATIONS) {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
      motorState = STOPPED;
      Serial.println("Target rotations reached. Motor stopped.");
    }
  }
  // --- STOPPED STATE ---
  else if (motorState == STOPPED) {
    // Ensure the motor remains stopped.
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
  }
  
  // Delay between control loop iterations (~100ms) to provide a simple update rate.
  delay(100);
}