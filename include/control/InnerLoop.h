#pragma once

#include "common/MathUtils.h"

class InnerLoop {
public:

	Vec<3> computeWrench(const Vec<3>& att_cmd,double yaw_rate_cmd, const Vec<3>& att, const Vec<3>& omega,double dt);

private:
	static const inline Vec<3> kp{ 0.200,0.200,0.0};

	static const inline Vec<3> kd{ 0.024,0.024,0.1 };
	
	static const inline Vec<3> ki{ 0.0,0.0,0.0 };

	double tauI = 0.025;
	double tauA = 0.015;

	Vec<3> x4 = Vec<3>::Zero();
	Vec<3> x5 = Vec<3>::Zero();

	static constexpr double Mx_max = 1.0;
	static constexpr double My_max = 1.0;
	static constexpr double Mz_max = 0.5;
};
