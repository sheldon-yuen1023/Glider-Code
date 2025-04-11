#include <SPI.h>
#include <HighPowerStepperDriver.h>

// ----------------------
// Pin Definitions
// ----------------------
const int limitSwitchHome = 5;       // Home limit switch input pin (pressed = HIGH)
const int limitSwitchMax  = 6;       // Maximum limit switch input pin (pressed = HIGH)
const int indicatorPin    = 41;      // Indicator output pin (optional)
const int potPin          = 18;      // Analog input for the linear potentiometer

// ----------------------
// Global Variables for Position
// ----------------------
int homePot   = 0;      // Pot reading at home (0 rotations)
int maxPot    = 0;      // Pot reading at maximum position
int centerPot = 0;      // Calculated center pot value
const int centerTolerance = 10;  // Acceptable error margin for center

// ----------------------
// Revolution Monitoring
// ----------------------
const long STEPS_PER_REV = 6400;  // Adjust for your motor (e.g., 200 steps * 32 microsteps)
long globalStepCount = 0;         // Tracks microsteps (used for rotation count)

// ----------------------
// State Machine Definitions
// ----------------------
enum InitState { 
  MOVE_REVERSE_HOME,  // Move in reverse until home switch is pressed
  MOVE_FORWARD_MAX,   // Move forward until maximum limit switch is pressed
  MOVE_TO_CENTER,     // Move toward center position based on pot value
  INIT_DONE           // Initialization complete; hold position
};

InitState initState = MOVE_REVERSE_HOME;

// ----------------------
// Motor Speed & Acceleration Settings
// ----------------------
const uint16_t initialDelayUs = 500;  // Starting delay after a direction change
const uint16_t targetDelayUs  = 100;  // Target (minimum) step delay for max speed
const uint16_t accelStep      = 5;    // How much to reduce the delay per step
uint16_t currentDelayUs       = initialDelayUs;  // Current delay between steps

// ----------------------
// Motor Direction Definitions
// ----------------------
const int FORWARD = 0;  // Forward motion
const int REVERSE = 1;  // Reverse motion
int currentDirection = REVERSE; // Start in reverse to find home

// ----------------------
// Motor Driver Setup
// ----------------------
const uint8_t CSPin = 10;         // Chip select pin for the stepper driver
HighPowerStepperDriver sd;        // Stepper driver instance

// ----------------------
// For Edge Detection on Limit Switches
// ----------------------
int prevHomeState = LOW;
int prevMaxState  = LOW;

void setup() {
  Serial.begin(115200);
  delay(100);  // Allow time for Serial monitor

  // Setup indicator pin (optional)
  pinMode(indicatorPin, OUTPUT);
  digitalWrite(indicatorPin, HIGH);

  // Configure limit switch pins as INPUT (assumes external circuitry so that pressed = HIGH)
  pinMode(limitSwitchHome, INPUT);
  pinMode(limitSwitchMax, INPUT);

  // Configure potentiometer input
  pinMode(potPin, INPUT);

  // ----------------------
  // Initialize the Motor Driver
  // ----------------------
  SPI.begin();
  sd.setChipSelectPin(CSPin);
  delay(10);  // Wait for power up

  sd.resetSettings();
  sd.clearStatus();
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  sd.setCurrentMilliamps36v4(3000);   // Adjust current limit as needed
  sd.setStepMode(HPSDStepMode::MicroStep32);
  sd.enableDriver();

  // Set initial direction to REVERSE (searching for home)
  currentDirection = REVERSE;
  sd.setDirection(currentDirection);

  // Initialize previous switch states (assume not pressed initially)
  prevHomeState = digitalRead(limitSwitchHome);
  prevMaxState  = digitalRead(limitSwitchMax);

  Serial.println("Pitch system initialization started...");
}

void loop() {
  // Read current limit switch states and potentiometer value
  int currHome  = digitalRead(limitSwitchHome);
  int currMax   = digitalRead(limitSwitchMax);
  int currPot   = analogRead(potPin);

  switch (initState) {
    
    case MOVE_REVERSE_HOME: {
      // Move in reverse until home limit switch is pressed.
      sd.step();
      delayMicroseconds(currentDelayUs);
      
      // Smooth acceleration: decrease delay gradually until reaching target speed.
      if(currentDelayUs > targetDelayUs)
        currentDelayUs = max((int)targetDelayUs, (int)(currentDelayUs - accelStep));
      
      // For reverse motion, decrement the global step counter.
      globalStepCount--;
      
      // Check for an edge: if the home switch goes from unpressed (LOW) to pressed (HIGH)
      if(currHome == HIGH && prevHomeState == LOW) {
        // Home found; record the pot reading and reset rotation count.
        homePot = currPot;
        globalStepCount = 0;
        Serial.print("Home reached (0 rotations). Home pot = ");
        Serial.println(homePot);
        
        // Transition to searching for the maximum position.
        initState = MOVE_FORWARD_MAX;
        currentDirection = FORWARD;
        sd.setDirection(currentDirection);
        // Reset ramp parameters.
        currentDelayUs = initialDelayUs;
      }
      break;
    }
    
    case MOVE_FORWARD_MAX: {
      // Move forward until maximum limit switch is pressed.
      sd.step();
      delayMicroseconds(currentDelayUs);
      if(currentDelayUs > targetDelayUs)
        currentDelayUs = max((int)targetDelayUs, (int)(currentDelayUs - accelStep));
      
      // For forward motion, increment the step counter.
      globalStepCount++;
      
      // Monitor full revolutions.
      if(globalStepCount >= STEPS_PER_REV) {
        long completeRevs = globalStepCount / STEPS_PER_REV;
        Serial.print("Complete revolutions (forward): ");
        Serial.println(completeRevs);
        globalStepCount = globalStepCount % STEPS_PER_REV;
      }
      
      // When maximum limit switch edge is detected (transition from unpressed LOW to pressed HIGH)
      if(currMax == HIGH && prevMaxState == LOW) {
        maxPot = currPot;
        Serial.print("Maximum position reached. Max pot = ");
        Serial.println(maxPot);
        
        // Calculate center based on the recorded home and maximum positions.
        centerPot = (homePot + maxPot) / 2;
        Serial.print("Calculated center pot value = ");
        Serial.println(centerPot);
        
        initState = MOVE_TO_CENTER;
        currentDelayUs = initialDelayUs;  // Reset ramp for the next phase.
      }
      break;
    }
    
    case MOVE_TO_CENTER: {
      // Use potentiometer feedback to move toward the center.
      int error = centerPot - currPot;
      
      // If within tolerance, hold position and mark initialization complete.
      if(abs(error) <= centerTolerance) {
        Serial.println("Center position reached. Initialization complete.");
        initState = INIT_DONE;
        break;
      }
      
      // Choose direction based on error: if error > 0, pot value is too low => move forward; else reverse.
      if(error > 0)
        currentDirection = FORWARD;
      else
        currentDirection = REVERSE;
      
      sd.setDirection(currentDirection);
      sd.step();
      delayMicroseconds(currentDelayUs);
      if(currentDelayUs > targetDelayUs)
        currentDelayUs = max((int)targetDelayUs, (int)(currentDelayUs - accelStep));
      
      // (Optional) Update step count if needed.
      if(currentDirection == FORWARD)
        globalStepCount++;
      else
        globalStepCount--;
      
      break;
    }
    
    case INIT_DONE: {
      // Hold position; initialization is complete. No further stepping.
      break;
    }
  }
  
  // Update previous switch states for edge detection.
  prevHomeState = currHome;
  prevMaxState  = currMax;
}