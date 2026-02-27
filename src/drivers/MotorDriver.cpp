#include "drivers/MotorDriver.h"
#include <algorithm>
#include <unistd.h>

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

bool MotorDriver::initialize() {
    for (int pin : motor_pins) {
        pwm_driver.initialize(pin);
        pwm_driver.set_frequency(pin, PWM_FREQ);
        usleep(50000);
        pwm_driver.enable(pin);
    }

    for (int pin : motor_pins) {
        pwm_driver.set_duty_cycle(pin, (float)PWM_MIN);
    }

    //calibrate();  shouldn't need this because it might cause the motors to command 2000 PWM if already calibrated. Need to test again. 

    usleep(50000);
    return true;
}

void MotorDriver::calibrate() {
    const int FEED_US = 50000; // 50ms
    // 5 seconds / 0.05 seconds = 100 loops
    const int loops = static_cast<int>(5.0 * 1000000 / FEED_US);

    // High Signal Max PWM
    for (int i = 0; i < loops; ++i) {
        for (int pin : motor_pins) {
            pwm_driver.set_duty_cycle(pin, (float)PWM_MAX);
        }
        usleep(FEED_US);
    }

    // Low Signal Min PWM
    for (int i = 0; i < loops; ++i) {
        for (int pin : motor_pins) {
            pwm_driver.set_duty_cycle(pin, (float)PWM_MIN);
        }
        usleep(FEED_US);
    }
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
