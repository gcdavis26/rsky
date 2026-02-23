#pragma once

#include "common/MathUtils.h"

class InnerLoop {
public:

	Vec<3> computeWrench(const Vec<3>& att_cmd,double yaw_rate_cmd, const Vec<3>& att, const Vec<3>& omega);

private:
	static const inline Vec<3> kp{ 2.0,2.0,2.0};

	static const inline Vec<3> kd{ 0.75,0.75,0.75 };

	static constexpr double Mx_max = 1.0;
	static constexpr double My_max = 1.0;
	static constexpr double Mz_max = 0.5;
};
