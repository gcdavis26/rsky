#pragma once

#include "common/MathUtils.h"

class InnerLoop {
public:

	Vec<3> computeWrench(const Vec<3>& att_cmd, const Vec<3>& att, const Vec<3>& omega);

private:
	static const inline Vec<3> kp{ 0.5,0.5,0.5 };

	static const inline Vec<3> kd{ 0.1,0.1,0.1 };

	static constexpr double Mx_max = 1.0;
	static constexpr double My_max = 1.0;
	static constexpr double Mz_max = 0.5;
};