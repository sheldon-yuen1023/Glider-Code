#include <Arduino.h>
#include "driver/mcpwm.h"  // Motor Control PWM

#define EN_PIN 35      // PWM control pin (connected to EN on driver)
#define PH_PIN 36      // Direction control pin (connected to PH on driver)

#define MCPWM_UNIT MCPWM_UNIT_0
#define MCPWM_TIMER MCPWM_TIMER_0
#define MCPWM_GEN MCPWM_GEN_A

void setup() {
    Serial.begin(115200);
    pinMode(PH_PIN, OUTPUT);  // Set direction pin as output

    // Initialize MCPWM on GPIO1
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, EN_PIN);

    // Configure MCPWM
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 5000;  // 5 kHz
    pwm_config.cmpr_a = 0;        // Initial duty cycle
    pwm_config.cmpr_b = 0;        // Not used
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT, MCPWM_TIMER, &pwm_config);
}

void accelerate(bool direction) {
    digitalWrite(PH_PIN, direction ? HIGH : LOW);
    Serial.println(direction ? "Accelerating Forward" : "Accelerating Reverse");

    for (int duty = 0; duty <= 100; duty += 2) {
        mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, duty);
        delay(100);
    }
}

void decelerate(bool direction) {
    digitalWrite(PH_PIN, direction ? HIGH : LOW);
    Serial.println(direction ? "Decelerating Forward" : "Decelerating Reverse");

    for (int duty = 100; duty >= 0; duty -= 2) {
        mcpwm_set_duty(MCPWM_UNIT, MCPWM_TIMER, MCPWM_GEN, duty);
        delay(100);
    }
}

void loop() {
    accelerate(true);    // Accelerate forward
    delay(1000);         // Hold full speed
    decelerate(true);    // Decelerate forward
    delay(500);

    accelerate(false);   // Accelerate reverse
    delay(1000);         // Hold full speed
    decelerate(false);   // Decelerate reverse
    delay(500);
}
