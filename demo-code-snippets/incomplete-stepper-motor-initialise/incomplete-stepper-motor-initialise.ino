#include <SPI.h>
#include <HighPowerStepperDriver.h>

// ----------------------
// Hardware Pin Definitions
// ----------------------
const uint8_t CSPin            = 10;   // Chip select pin for the stepper driver
const int limitSwitchHomePin   = 5;    // Home limit switch (pressed = HIGH)
const int limitSwitchMaxPin    = 6;    // Maximum position limit switch (pressed = HIGH)
const int potPin               = 18;   // Analog input pin for the linear potentiometer
const int indicatorPin         = 41;   // Optional indicator LED pin

// ----------------------
// Stepper and Motion Settings
// ----------------------
const uint16_t TARGET_STEP_DELAY = 1000;   // Base delay in microseconds between steps at constant speed

// Define your microstep count for one complete revolution.
// For example, if the motor is 200 full steps/rev and microstepping is set to 16:
const long STEPS_PER_REV = 256 * 16;       // = 3200 microsteps per revolution

// ----------------------
// Smooth Motion Parameters (for moves that need acceleration/deceleration)
// ----------------------
const uint16_t initialDelay = 1200;   // Starting delay (slower speed) after a direction change (µs)
const uint16_t targetDelay  = TARGET_STEP_DELAY;  // Final, steady delay (µs)
const uint16_t rampSteps    = 50;     // Number of steps over which to ramp acceleration/deceleration

// ----------------------
// Global Variables for Position Tracking
// ----------------------
long globalStepCount = 0;  // Tracks the current position in microsteps (relative to home)
long homeSteps = 0;        // Home position (should be 0)
long maxSteps  = 0;        // Recorded position when maximum limit switch is triggered

// Record the potentiometer readings at the two limits
int homePot = 0;
int maxPot  = 0;

// ----------------------
// State Machine for Initialization
// ----------------------
enum InitState { SEARCH_HOME, SEARCH_MAX, MOVE_TO_CENTER, INIT_COMPLETE };
InitState initState = SEARCH_HOME;

// ----------------------
// Create Stepper Driver Instance
// ----------------------
HighPowerStepperDriver sd;

// ----------------------
// Function: smoothMoveSteps
// ----------------------
// Moves the motor a given number of microsteps smoothly using a simple ramp acceleration profile.
// Positive steps move forward; negative steps move reverse.
void smoothMoveSteps(long steps) {
  bool forward = (steps > 0);
  long totalSteps = abs(steps);
  
  // Determine ramp (acceleration) steps.
  long accelSteps = min((long)rampSteps, totalSteps / 2);
  long decelSteps = accelSteps;
  long constSteps = totalSteps - accelSteps - decelSteps;

  uint16_t currentDelay = initialDelay;
  
  // --- Acceleration Phase ---
  for (long i = 0; i < accelSteps; i++) {
    sd.step();
    delayMicroseconds(currentDelay);
    if (forward) {
      globalStepCount++;
    } else {
      globalStepCount--;
    }
    // Linearly reduce delay toward targetDelay.
    currentDelay = initialDelay - ((initialDelay - targetDelay) * (i + 1)) / accelSteps;
  }
  
  // --- Constant Speed Phase ---
  for (long i = 0; i < constSteps; i++) {
    sd.step();
    delayMicroseconds(targetDelay);
    if (forward) {
      globalStepCount++;
    } else {
      globalStepCount--;
    }
  }
  
  // --- Deceleration Phase ---
  currentDelay = targetDelay;
  for (long i = 0; i < decelSteps; i++) {
    sd.step();
    delayMicroseconds(currentDelay);
    if (forward) {
      globalStepCount++;
    } else {
      globalStepCount--;
    }
    // Linearly increase delay toward initialDelay.
    currentDelay = targetDelay + ((initialDelay - targetDelay) * (i + 1)) / decelSteps;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100); // Allow time for Serial to initialize

  // Initialize optional indicator LED
  pinMode(indicatorPin, OUTPUT);
  digitalWrite(indicatorPin, HIGH);

  // Configure limit switches as INPUT_PULLUP (if using pull-ups, otherwise adjust wiring)
  pinMode(limitSwitchHomePin, INPUT_PULLUP);
  pinMode(limitSwitchMaxPin, INPUT_PULLUP);

  // Configure the potentiometer input
  pinMode(potPin, INPUT);

  // ----------------------
  // Motor Driver Initialization (using your final SPI code settings)
  // ----------------------
  SPI.begin();
  sd.setChipSelectPin(CSPin);
  delay(10); // Allow driver time to power up

  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  sd.setCurrentMilliamps36v4(4000);      // Adjust according to your motor's rating
  sd.setStepMode(HPSDStepMode::MicroStep16);  // Set to MicroStep16 mode per your final code
  sd.enableDriver();

  // Start by searching for the home position.
  // Set direction to reverse (assume: 1 = reverse, 0 = forward)
  sd.setDirection(1);
  Serial.println("Initialization: Searching for Home position...");
}

void loop() {
  switch (initState) {

    case SEARCH_HOME: {
      // Drive the motor in reverse until the home limit switch is pressed.
      // Now, a pressed switch gives HIGH.
      if (digitalRead(limitSwitchHomePin) == HIGH) {  // Home switch pressed
        // Record the potentiometer reading as home.
        homePot = analogRead(potPin);
        Serial.print("Home limit reached. Recorded pot value: ");
        Serial.println(homePot);
        // Set home position as 0 microsteps.
        globalStepCount = 0;
        homeSteps = 0;
        Serial.println("Switching direction; now searching for Maximum position...");
        // Change direction to forward.
        sd.setDirection(0);
        initState = SEARCH_MAX;
      } else {
        // While searching home, run at constant speed.
        sd.step();
        delayMicroseconds(TARGET_STEP_DELAY);
        // Since moving in reverse, decrement the global step counter.
        globalStepCount--;
      }
      break;
    }

    case SEARCH_MAX: {
      // Drive the motor forward until the maximum limit switch is pressed.
      if (digitalRead(limitSwitchMaxPin) == HIGH) {  // Maximum switch pressed
        maxPot = analogRead(potPin);
        Serial.print("Maximum limit reached. Recorded pot value: ");
        Serial.println(maxPot);
        // Record the maximum step count reached from home.
        maxSteps = globalStepCount;
        Serial.print("Total microsteps from home to maximum: ");
        Serial.println(maxSteps);
        Serial.print("Complete revolutions (forward): ");
        Serial.println((float)maxSteps / STEPS_PER_REV);
        initState = MOVE_TO_CENTER;
      } else {
        sd.step();
        delayMicroseconds(TARGET_STEP_DELAY);
        globalStepCount++;
      }
      break;
    }

    case MOVE_TO_CENTER: {
      // Compute center position as half the distance from home (0) to max.
      long centerSteps = maxSteps / 2;
      long stepsToMove = centerSteps - globalStepCount;
      Serial.print("Centering: Need to move ");
      Serial.print(stepsToMove);
      Serial.println(" microsteps to reach center.");
      
      if (stepsToMove != 0) {
        if (stepsToMove > 0) {
          sd.setDirection(0);  // Move forward
        } else {
          sd.setDirection(1);  // Move reverse
        }
        smoothMoveSteps(stepsToMove);
      } else {
        Serial.println("Already at center.");
      }
      initState = INIT_COMPLETE;
      break;
    }

    case INIT_COMPLETE: {
      Serial.println("Pitch system initialization complete.");
      // Hold position indefinitely.
      while (1) {
        delay(1000);
      }
      break;
    }
  }
}

