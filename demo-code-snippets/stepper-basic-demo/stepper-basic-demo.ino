// This example shows basic use of a Pololu High Power Stepper Motor Driver.
//
// It shows how to initialize the driver, configure various settings, and enable
// the driver.  It shows how to step the motor and switch directions using the
// driver's SPI interface through the library's step() and setDirection() member
// functions.
//
// Since SPI is used to trigger steps and set the direction, connecting the
// driver's STEP and DIR pins are not needed for this example.  Note that
// using SPI control adds some overhead compared to using the STEP and DIR pins.
// In addition, since the library caches SPI register values, SPI control is
// more likely to re-enable the driver with the wrong settings (e.g. current
// limit) after a power interruption, although using the verifySettings() and
// applySettings() functions appropriately can help prevent this.
//
// Before using this example, be sure to change the setCurrentMilliamps36v4 line
// to have an appropriate current limit for your system.  Also, see this
// library's documentation for information about how to connect the driver:
//   https://pololu.github.io/high-power-stepper-driver-arduino/

#include <SPI.h>
#include <HighPowerStepperDriver.h>

const uint8_t CSPin = 10;

// Reduce delay between steps to increase speed (e.g., 1000 µs = 1 kHz step rate)
const uint16_t StepPeriodUs = 1000;

HighPowerStepperDriver sd;

void setup()
{
  SPI.begin();
  sd.setChipSelectPin(CSPin);

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver and clear any latched faults
  sd.resetSettings();
  sd.clearStatus();

  // Use auto mixed decay mode
  sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set current limit (tweak this based on your stepper's rating)
  sd.setCurrentMilliamps36v4(600);

  // Set microstepping mode – you can lower this to MicroStep16 or MicroStep8 to further increase speed
  sd.setStepMode(HPSDStepMode::MicroStep16);

  // Enable the motor outputs
  sd.enableDriver();
}

void loop()
{
  // Step forward 1000 steps
  sd.setDirection(0);
  for (unsigned int x = 0; x < 5000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  delay(300);

  // Step backward 1000 steps
  sd.setDirection(1);
  for (unsigned int x = 0; x < 5000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  delay(300);
}
