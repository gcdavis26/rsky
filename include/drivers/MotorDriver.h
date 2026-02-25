#pragma once

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <vector>
#include <Eigen/Dense>
#include "Navio2/RCOutput_Navio2.h"

#define NUM_MOTORS 4
#define PWM_FREQ 400
#define PWM_MIN 1000.0
#define PWM_MAX 2000.0 // 2000 normally, 1400 for safety.
#define PWM_SAFE 1000.0

class MotorDriver {
public:
    MotorDriver();
    ~MotorDriver();

    bool initialize();
    void calibrate(); // Add this line

    void command(const Eigen::Vector4d& pwm_values);

    void wind_down();

private:
    RCOutput_Navio2 pwm_driver;
    const std::vector<int> motor_pins = { 0, 1, 2, 3 }; //
};

#endif