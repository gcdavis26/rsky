#pragma once

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <vector>
#include <Eigen/Dense>
#include "Navio2/RCOutput_Navio2.h"
#include "common/MathUtils.h"

#define NUM_MOTORS 4
#define PWM_FREQ 400
#define PWM_MIN 1000.0
#define PWM_MAX 2000.0 // 2000 normally, 1400 for safety.
#define PWM_SAFE 1150.0

class MotorDriver {
public:
    MotorDriver(int calib);
    ~MotorDriver();

    bool initialize();
    void calibrate(); // Add this line

    void arm();
    void disarm();
    bool isArmed() const;

    void command(const Vec<4>& pwm_values);
    void commandServo(double pwm_values);

    void wind_down();

private:
    bool armed = false;
    RCOutput_Navio2 pwm_driver;
    const std::vector<int> motor_pins = { 0, 1, 2, 3}; //
};

#endif
