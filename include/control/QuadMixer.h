#pragma once
#include "common/MathUtils.h"

class QuadMixer {
public:
	Vec<4> mix2Thrust(const Vec<4>& cmd);
	Vec<4> mix2Wrench(const Vec<4>& T);
	Vec<4> thr2PWM(const Vec<4>& thrust_cmd);
private:
	Vec<4> clampThrusts(const Vec<4>& T);

	double arm_length = 0.25;
	double k_yaw = 0.02;
	double T_min = 0.0;
	double T_max = 2.0 * mass * g / 4.0;

	const double thrust_min = 0.0;
	const double thrust_max = 2 * mass * g / 4;
	const double e = 0.01;
	const double PWM_MIN = 1000;
	const double PWM_MAX = 2000;
};