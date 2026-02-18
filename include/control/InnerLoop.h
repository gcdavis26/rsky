#pragma once

#include "common/MathUtils.h"

class InnerLoop {
public:

	Vec<3> computeWrench(const Vec<3>& att_cmd,double yaw_rate_cmd, const Vec<3>& att, const Vec<3>& omega);

private:
	static const inline Vec<3> kp{ 0.3,0.3,0.3 };

	static const inline Vec<3> kd{ 0.25,0.25,0.25 };

	static constexpr double Mx_max = 1.0;
	static constexpr double My_max = 1.0;
	static constexpr double Mz_max = 0.5;
};