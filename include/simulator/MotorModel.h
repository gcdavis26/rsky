#pragma once
#include "common//MathUtils.h"

class MotorModel {
public:
	void arm();
	void disarm();
	bool isArmed() const;

	Vec<4> step(double dt, const Vec<4>& thrust_cmd);

	// Stub functions to match MotorDriver interface for MotorTask template
	void command(const Vec<4>& /*pwm_values*/) {}
	void commandServo(double /*pwm_servo*/) {}
	void wind_down() {}

private:
	bool armed = false;
	double tau = 0.05;
	Vec<4> thrustAct = Vec<4>::Zero();
};
