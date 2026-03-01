#include "control/QuadMixer.h"


Vec<4> QuadMixer::mix2Thrust(const Vec<4>& cmd) {

	const double L = arm_length;
	const double a = L / 1.41421356237;
	const double k = k_yaw;

	const double F = cmd(0);
	const double Rx = cmd(1) / a;
	const double Ry = cmd(2)/ a;
	const double Y = cmd(3) / k;

	Vec<4> T;

	T(0) = 0.25 * (F + Rx + Ry + Y); // T1
	T(1) = 0.25 * (F - Rx + Ry - Y); // T2
	T(2) = 0.25 * (F - Rx - Ry + Y); // T3
	T(3) = 0.25 * (F + Rx - Ry - Y); // T4

	return clampThrusts(T);
}

Vec<4> QuadMixer::mix2Wrench(const Vec<4>& T) {
	const double L = arm_length;
	const double a = L / 1.41421356237;
	const double k = k_yaw;

	Vec<4> w;
	w(0) = T(0) + T(1) + T(2) + T(3);
	w(1) = a * (T(0) - T(1) - T(2) + T(3));
	w(2) = a * (T(0) + T(1) - T(2) - T(3));
	w(3) = k * (T(0) - T(1) + T(2) - T(3));

	return w;
}


Vec<4> QuadMixer::clampThrusts(const Vec<4>& T) {
	Vec<4> Tc = T;
	Tc(0) = clamp(Tc(0), T_min, T_max);
	Tc(1) = clamp(Tc(1), T_min, T_max);
	Tc(2) = clamp(Tc(2), T_min, T_max);
	Tc(3) = clamp(Tc(3), T_min, T_max);
	return Tc;
}

Vec<4> QuadMixer::thr2PWM(const Vec<4>& thrust_cmd) {
	Vec<4> PWM = Vec<4>::Zero();
	

	for (int i = 0; i < 4; i++) {

		PWM(i) = (0.0468 + sqrt(2.226 * 0.00001 + 8.88 * 0.00001 * thrust_cmd(i))) / (4.44 * 0.00001);

		PWM(i) = clamp(PWM(i), 1000.0, 2000.0);
	}
	return PWM;
}