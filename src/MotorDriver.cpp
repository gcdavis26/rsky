#include "MotorDriver.h"
#include <algorithm>

// For Main
// Constuctor Line: MotorDriver motors;
// Initialisation Line: motors.initialize();
// Set Motor PWM via 4x1 Eigen Matrix: motors.command("4x1 PWM Matrix")
// End Code control over motors at the end: motors.wind_down()


MotorDriver::MotorDriver() {
    // Direct instantiation for Navio2
}

MotorDriver::~MotorDriver() {
    wind_down();
}

void MotorDriver::initialize() {
    for (int pin : motor_pins) {
        pwm_driver.initialize(pin);
        pwm_driver.set_frequency(pin, PWM_FREQ);
        pwm_driver.enable(pin);
    }

    // Arming sequence: ensure ESCs see low signal to initialize
    for (int pin : motor_pins) {
        pwm_driver.set_duty_cycle(pin, (float)PWM_SAFE);
    }

    sleep(0.2);
}

void MotorDriver::command(const Eigen::Vector4d& pwm_values) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        // Clamp the double input and cast to float for the hardware driver
        double commanded = std::clamp(pwm_values(i), (double)PWM_MIN, (double)PWM_MAX);

        pwm_driver.set_duty_cycle(motor_pins[i], static_cast<float>(commanded));
    }
}

void MotorDriver::wind_down() 
{
    for (int pin : motor_pins) {
        pwm_driver.set_duty_cycle(pin, (float)PWM_MIN);
    }
    usleep(50000);
}

Eigen::Vector4d throttle2pwm(Eigen::Vector4d throttles)
{
    Eigen::Vector4d pwms = (throttles.array() + 1.0) * 1000;
    return pwms;
}