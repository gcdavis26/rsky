#include "simulator/MotorModel.h"

Vec<4> MotorModel::step(double dt, const Vec<4>& thrust_cmd) {
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
