#include <Arduino.h>
#include "driver/mcpwm.h"  // Motor Control PWM

// Motor control pins
#define EN_PIN 16             // PWM control pin (connected to EN on driver)
#define PH_PIN 15             // Direction control pin (connected to PH on driver)
#define LIMIT_SWITCH_PIN 18   // Limit switch pin

// Encoder pins (adjust these GPIO numbers as needed)
#define ENC_A_PIN 12           // Encoder Channel A
#define ENC_B_PIN 13           // Encoder Channel B
#define ENC_IDX_PIN 14         // Optional: Encoder Index

volatile long encoderCount = 0;         // Global encoder pulse count
const int countsPerRevolution = 1024;     // Encoder resolution (counts per revolution)

// Motor forward rotation target: set number of forward revolutions (e.g., 2 revolutions)
const float TARGET_ROTATIONS = 200000.0;

// Define a simple state machine for motor control
enum MotorState {
  REVERSING,      // Motor runs in reverse until limit switch is pressed
  MOVING_FORWARD, // Motor runs forward until target rotations are reached
  STOPPED         // Motor stops once the target is met
};

MotorState motorState = REVERSING;

// Interrupt Service Routine for encoder channel A
// This ISR reads channel B to determine the direction of rotation.
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

  // Setup motor control pins
  pinMode(PH_PIN, OUTPUT);
  // Setup limit switch as input with internal pullup.
  // For this configuration, the limit switch outputs HIGH when pressed.
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  
  // Start in reverse mode.
  // In this example, assume forward direction is HIGH and reverse is LOW.
  digitalWrite(PH_PIN, HIGH);

  // Initialize MCPWM for motor speed control
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;        // 5 kHz PWM frequency
  pwm_config.cmpr_a = 80.0;           // 80% duty cycle (adjust as needed)
  pwm_config.cmpr_b = 0.0;            
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Setup encoder pins as inputs with internal pull-ups
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_IDX_PIN, INPUT_PULLUP);  // Optional: use if you need an index pulse

  // Attach interrupt on rising edge of encoder Channel A to count pulses.
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  // Optional short delay to stabilize start-up
  delay(100);
}

void loop() {
  // --- REVERSING STATE ---
  // Run motor in reverse until the limit switch is pressed.
  if (motorState == REVERSING) {
    // Check if the limit switch has been activated.
    // Now the limit switch outputs HIGH when pressed.
    if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
      // Limit switch pressed:
      // 1. Atomically reset the encoder to zero.
      noInterrupts();
      encoderCount = 0;
      interrupts();
      
      Serial.println("Limit switch pressed. Encoder reset. Changing direction to FORWARD.");
      
      // 2. Change motor direction to forward.
      digitalWrite(PH_PIN, LOW);
      
      // 3. Move to next state.
      motorState = MOVING_FORWARD;
    }
  }
  // --- MOVING_FORWARD STATE ---
  // Motor runs in forward direction. Count rotations using encoderCount.
  else if (motorState == MOVING_FORWARD) {
    // Calculate rotations (as a float) based on encoder counts.
    float rotations = (float)encoderCount / countsPerRevolution;
    
    Serial.print("Forward Rotations: ");
    Serial.println(rotations);
    
    // If the measured forward rotations reach or exceed the target, stop the motor.
    if (rotations >= TARGET_ROTATIONS) {
      // Set PWM duty cycle to 0 to stop the motor.
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
      motorState = STOPPED;
      Serial.println("Target rotations reached. Motor stopped.");
    }
  }
  // --- STOPPED STATE ---
  // Once stopped, you may add further code here or simply do nothing.
  else if (motorState == STOPPED) {
    // Motor is stopped; no action is taken.
  }
  
  // A small delay to avoid flooding the Serial output.
  delay(100);
}