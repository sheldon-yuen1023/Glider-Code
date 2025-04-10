#include <Arduino.h>
#include "driver/mcpwm.h"

// ----- PIN DEFINITIONS -----
// Motor driver pins
#define EN_PIN             15    // PWM control output (to EN on motor driver)
#define PH_PIN             16    // Direction control (set HIGH for reverse, LOW for forward)

// Limit switch input (detects piston home position)
#define LIMIT_SWITCH_PIN   8     // Uses internal pullup (active when pressed)

// Encoder input pins
#define ENC_A_PIN          12    // Encoder Channel A (with interrupt)
#define ENC_B_PIN          13    // Encoder Channel B
#define ENC_IDX_PIN        14    // Optional index pulse (if used)

// Leak sensor input
#define LEAK_SENSOR_PIN    40    // Digital input: HIGH indicates leak

// UART pins for Flight Controller communication (using HardwareSerial2)
#define UART_RX_PIN        18    
#define UART_TX_PIN        17    

// ----- GLOBALS & CONSTANTS -----
//
// Create an instance of HardwareSerial on UART2.
HardwareSerial mySerial(2);

// Global stop flag (set when STOP command is received or when leak is detected)
volatile bool stopFlag = false;

// New flag to ensure emergency surface triggers only once
volatile bool emergencyTriggered = false;

// Encoder count (updated in ISR)
volatile long encoderCount = 0;
const long ENCODER_RESOLUTION = 159744;  // Encoder pulses per full revolution

// System parameters (rotations)
const int NEUTRAL_ROTATIONS = 3;         // # Rotations corresponding to neutrally buoyant state
const float maxRotations = 6;            // # Rotations corresponding to full extension (fixed limit)

// Motor duty cycle settings (maximum values)
const float FORWARD_DUTY = 10.0;   // Maximum duty cycle for extending piston
const float REVERSE_DUTY = 10.0;   // Maximum duty cycle for retracting piston

// Acceleration and deceleration parameters (tweak as needed)
const float ACCEL_STEP = 1.0;               // PWM duty increment per control loop iteration
const float DECEL_STEP = 1.0;               // PWM duty decrement per control loop iteration
const float DECELERATION_WINDOW = 1.0;      // Window (in revolutions) within which deceleration begins

// ----- FUNCTION PROTOTYPES -----
void encoderISR();
long safeReadEncoder();
void setupMCPWM();
void stopMotor();

void zeroEncoder();                // Retract piston until limit switch triggers
void initialiseSystem();           // Zero the encoder and move to neutrally buoyant position
void moveToRotations(int targetRotations);
void moveExtend(long targetEncoderCount);
void moveRetract(long targetEncoderCount);
void startMission(int cycles);
void waitForTargetDepth();         // Wait for "TARGET_DEPTH_REACHED" command (dive)
void waitForTargetDepth2();        // Wait for "TARGET_DEPTH_REACHED2" command (surface)
void emergencySurface();           // Emergency surfacing routine (if leak detected)

// ----- SETUP -----
void setup() {
  Serial.begin(115200);             // Debug output to USB serial

  // Initialize UART (HardwareSerial2) for communication with the Flight Controller.
  mySerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Configure motor control and sensor pins
  pinMode(PH_PIN, OUTPUT);
  digitalWrite(PH_PIN, LOW);        // Default to forward

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(LEAK_SENSOR_PIN, INPUT);

  // Setup encoder inputs with pull-ups.
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_IDX_PIN, INPUT_PULLUP);

  // Attach interrupt for encoder channel A.
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  // Initialize MCPWM for motor speed control (5 kHz PWM frequency)
  setupMCPWM();

  Serial.println("System initialised. Awaiting commands...");
  mySerial.println("SYSTEM_READY");
}

// ----- MCPWM SETUP -----
void setupMCPWM() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, EN_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000;        // 5 kHz PWM frequency
  pwm_config.cmpr_a = 0.0;              // Start with 0% duty cycle
  pwm_config.cmpr_b = 0.0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

// ----- ENCODER ISR -----
// Called on rising edge of encoder channel A; uses channel B state to determine direction.
void IRAM_ATTR encoderISR() {
  int bState = digitalRead(ENC_B_PIN);
  if (bState == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ----- Utility: Safe read of encoder count (disable interrupts briefly) -----
long safeReadEncoder() {
  long count;
  noInterrupts();
  count = encoderCount;
  interrupts();
  return count;
}

// ----- Stop Motor Function -----
// Immediately stops the motor by setting PWM duty to 0.
void stopMotor() {
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
  mySerial.println("MOTOR_STOPPED");
}

// ----- Function: ZERO_ENCODER -----
// Retracts the piston with a fixed reverse duty until the limit switch triggers,
// then resets the encoder count. (Home position)
void zeroEncoder() {
  mySerial.println("ZEROING_ENCODER");
  
  // Reset stop flag for new operation
  stopFlag = false;
  
  // Set motor direction to reverse (PH_PIN HIGH selects reverse).
  digitalWrite(PH_PIN, HIGH);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, REVERSE_DUTY);

  while (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
    if (stopFlag) { 
      stopMotor();
      return;
    }
    if (digitalRead(LEAK_SENSOR_PIN) == HIGH) {
      emergencySurface();
      return;
    }
    delay(10);
  }
  
  stopMotor();
  noInterrupts();
  encoderCount = 0;
  interrupts();
  mySerial.println("HOME_POSITION");
}

// ----- Function: INITIALISE -----
// Zeros the encoder and then extends the piston to the neutrally buoyant position.
void initialiseSystem() {
  stopFlag = false;  // Reset stop flag for new command
  zeroEncoder();
  moveToRotations(NEUTRAL_ROTATIONS);
  mySerial.println("INITIALISE_COMPLETE");
}

// ----- Function: MOVE_TO_ROTATIONS -----
// Moves the piston to achieve the specified number of rotations. After completion,
// it sends a status message based on the target position.
void moveToRotations(int targetRotations) {
  stopFlag = false;  // Reset stop flag for new motion

  if (targetRotations > maxRotations) {
    targetRotations = (int)maxRotations;
  }
  
  long targetEncoderCount = (long)targetRotations * ENCODER_RESOLUTION;
  long currentPosition = safeReadEncoder();
  
  if (targetEncoderCount > currentPosition) {
    moveExtend(targetEncoderCount);
  } else if (targetEncoderCount < currentPosition) {
    moveRetract(targetEncoderCount);
  } else {
    stopMotor();
  }

  // Send the appropriate final position message.
  if (targetRotations == 0) {
    mySerial.println("HOME_POSITION");
  } else if (targetRotations == NEUTRAL_ROTATIONS) {
    mySerial.println("NEUTRAL_POSITION");
  } else if (targetRotations == (int)maxRotations) {
    mySerial.println("FULLY_EXTENDED");
  }
}

// ----- Function: MOVE_EXTEND -----
// Extends the piston forward using gradual acceleration and deceleration.
void moveExtend(long targetEncoderCount) {
  float currentDuty = 0.0;
  digitalWrite(PH_PIN, LOW);  // Set motor to forward
  
  while (safeReadEncoder() < targetEncoderCount) {
    if (stopFlag) { 
      stopMotor();
      return; 
    }
    // Only check leak sensor if emergency not already triggered.
    if (!emergencyTriggered && digitalRead(LEAK_SENSOR_PIN) == HIGH) {
      emergencySurface();
      return;
    }
    
    long currentEnc = safeReadEncoder();
    long distanceRemaining = targetEncoderCount - currentEnc;
    float threshold = DECELERATION_WINDOW * ENCODER_RESOLUTION;
    float targetDuty;
    
    // Decelerate if nearing the target.
    if (distanceRemaining <= threshold) {
      targetDuty = ((float)distanceRemaining / threshold) * FORWARD_DUTY;
      if (targetDuty < 10.0 && distanceRemaining > 0)
        targetDuty = 10.0;
    } else {
      targetDuty = FORWARD_DUTY;
    }
    
    if (currentDuty < targetDuty) {
      currentDuty += ACCEL_STEP;
      if (currentDuty > targetDuty)
        currentDuty = targetDuty;
    } else if (currentDuty > targetDuty) {
      currentDuty -= DECEL_STEP;
      if (currentDuty < targetDuty)
        currentDuty = targetDuty;
    }
    
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentDuty);
    delay(10);
  }
  
  stopMotor();
}

// ----- Function: MOVE_RETRACT -----
// Retracts the piston (reverse) using gradual acceleration and deceleration.
void moveRetract(long targetEncoderCount) {
  float currentDuty = 0.0;
  digitalWrite(PH_PIN, HIGH);  // Set motor to reverse
  
  while (safeReadEncoder() > targetEncoderCount) {
    if (stopFlag) { 
      stopMotor();
      return; 
    }
    // Only check leak sensor if emergency not already triggered.
    if (!emergencyTriggered && digitalRead(LEAK_SENSOR_PIN) == HIGH) {
      emergencySurface();
      return;
    }
    
    long currentEnc = safeReadEncoder();
    long distanceRemaining = currentEnc - targetEncoderCount;
    float threshold = DECELERATION_WINDOW * ENCODER_RESOLUTION;
    float targetDuty;
    
    if (distanceRemaining <= threshold) {
      targetDuty = ((float)distanceRemaining / threshold) * REVERSE_DUTY;
      if (targetDuty < 10.0 && distanceRemaining > 0)
        targetDuty = 10.0;
    } else {
      targetDuty = REVERSE_DUTY;
    }
    
    if (currentDuty < targetDuty) {
      currentDuty += ACCEL_STEP;
      if (currentDuty > targetDuty)
        currentDuty = targetDuty;
    } else if (currentDuty > targetDuty) {
      currentDuty -= DECEL_STEP;
      if (currentDuty < targetDuty)
        currentDuty = targetDuty;
    }
    
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentDuty);
    delay(10);
  }
  
  stopMotor();
}

// ----- Function: WAIT_FOR_TARGET_DEPTH -----
// Waits for the "TARGET_DEPTH_REACHED" command (used during dive).
void waitForTargetDepth() {
  mySerial.println("WAITING_FOR_TARGET_DEPTH");
  while (true) {
    if (stopFlag) { 
      stopMotor();
      return; 
    }
    if (mySerial.available() > 0) {
      String msg = mySerial.readStringUntil('\n');
      msg.trim();
      Serial.print("Received (dive): ");
      Serial.println(msg);
      if (msg.indexOf("TARGET_DEPTH_REACHED") >= 0) {
        mySerial.println("TARGET_DEPTH_ACKNOWLEDGED");
        break;
      }
    }
    if (!emergencyTriggered && digitalRead(LEAK_SENSOR_PIN) == HIGH) {
      emergencySurface();
      break;
    }
    delay(10);
  }
}

// ----- Function: WAIT_FOR_TARGET_DEPTH2 -----
// Waits for the "TARGET_DEPTH_REACHED2" command (used when at surface).
void waitForTargetDepth2() {
  mySerial.println("WAITING_FOR_TARGET_DEPTH_2");
  while (true) {
    if (stopFlag) { 
      stopMotor();
      return; 
    }
    if (mySerial.available() > 0) {
      String msg = mySerial.readStringUntil('\n');
      msg.trim();
      Serial.print("Received (surface): ");
      Serial.println(msg);
      if (msg.indexOf("TARGET_DEPTH_REACHED2") >= 0) {
        mySerial.println("TARGET_DEPTH_2_ACKNOWLEDGED");
        break;
      }
    }
    if (!emergencyTriggered && digitalRead(LEAK_SENSOR_PIN) == HIGH) {
      emergencySurface();
      break;
    }
    delay(10);
  }
}

// ----- Function: START_MISSION -----
// Executes a series of dive/ascent cycles. Each cycle uses gradual acceleration during both dive and ascent.
void startMission(int cycles) {
  stopFlag = false;  // Clear previous stop flag
  
  for (int cycle = 1; cycle <= cycles; cycle++) {
    if (stopFlag) break;
    mySerial.print("STARTING_CYCLE_");
    mySerial.println(cycle);
    
    // Dive: Retract piston to home (0 rotations) then wait for depth command.
    moveToRotations(0);
    waitForTargetDepth();
    if (stopFlag) break;
    
    // Ascent: Extend piston to neutrally buoyant position then wait for surface depth command.
    moveToRotations(NEUTRAL_ROTATIONS);
    waitForTargetDepth2();
    
    mySerial.print("CYCLE_COMPLETE_");
    mySerial.println(cycle);
  }
  mySerial.println("MISSION_COMPLETE");
}

// ----- Function: EMERGENCY_SURFACE -----
// Immediately stops any current movement and extends the piston fully if a leak is detected.
// Two emergency surface messages are sent to the flight controller.
// Once full extension is reached, leak sensor checks are no longer performed.
void emergencySurface() {
  // Check if already triggered to avoid repeated calls.
  if (emergencyTriggered) return;
  emergencyTriggered = true;
  
  // Immediately halt any movement.
  stopFlag = true;
  stopMotor();
  
  // Notify flight controller that emergency surfacing has been triggered (printed twice).
  mySerial.println("EMERGENCY_SURFACE_TRIGGERED");
  mySerial.println("EMERGENCY_SURFACE_TRIGGERED");

  // Command a full extension.
  long targetEncoderCount = (long)maxRotations * ENCODER_RESOLUTION;
  
  // Set motor to forward direction.
  digitalWrite(PH_PIN, LOW);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FORWARD_DUTY);
  
  // Continue extending until full extension is reached without checking the leak sensor.
  while (safeReadEncoder() < targetEncoderCount) {
    delay(10);
  }
  
  stopMotor();
  mySerial.println("FULLY_EXTENDED");
}

// ----- MAIN LOOP -----
// Monitors incoming commands and the leak sensor. The STOP command has highest priority.
void loop() {
  // Check leak sensor only if emergency hasn't already been triggered.
  if (!emergencyTriggered && digitalRead(LEAK_SENSOR_PIN) == HIGH) {
    emergencySurface();
    while (mySerial.available()) {
      mySerial.read();
    }
    return;
  }
  
  if (mySerial.available()) {
    String command = mySerial.readStringUntil('\n');
    command.trim();
    
    // If STOP command is received, set the flag and stop.
    if (command == "STOP") {
      stopFlag = true;
      stopMotor();
      mySerial.println("STOP_COMMAND_RECEIVED");
    } 
    else if (command == "INITIALISE") {
      stopFlag = false;
      initialiseSystem();
    } 
    else if (command.startsWith("START MISSION")) {
      stopFlag = false;
      int spaceIndex = command.lastIndexOf(' ');
      if (spaceIndex > 0) {
        int cycles = command.substring(spaceIndex + 1).toInt();
        if (cycles > 0) {
          startMission(cycles);
        } else {
          mySerial.println("INVALID_CYCLE_COUNT");
        }
      } else {
        mySerial.println("INVALID_COMMAND_FORMAT");
      }
    } 
    else if (command.startsWith("MOVE_TO")) {
      stopFlag = false;
      int spaceIndex = command.lastIndexOf(' ');
      if (spaceIndex > 0) {
        int targetRot = command.substring(spaceIndex + 1).toInt();
        moveToRotations(targetRot);
      } else {
        mySerial.println("INVALID_COMMAND_FORMAT");
      }
    } 
    else {
      mySerial.println("INVALID_COMMAND");
    }
  }
  delay(10);
}