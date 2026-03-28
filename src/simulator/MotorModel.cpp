#include "simulator/MotorModel.h"

void MotorModel::arm() {
	armed = true;
}

void MotorModel::disarm() {
	if (armed) {
		armed = false;
		thrustAct = Vec<4>::Zero();
	}
}

bool MotorModel::isArmed() const {
	return armed;
}

Vec<4> MotorModel::step(double dt, const Vec<4>& thrust_cmd) {
	if (!armed) {
		thrustAct = Vec<4>::Zero();
		return thrustAct;
	}

	if (tau <= 1e-9) {
		thrustAct = thrust_cmd;
		return thrustAct;
	}
	double alpha = dt / tau;
	if (alpha > 1.0) {
		alpha = 1.0;
	}

	thrustAct = thrustAct + alpha * (thrust_cmd - thrustAct);
	return thrustAct;
}
