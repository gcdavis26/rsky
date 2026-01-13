#pragma once

#include "common/types.h"

namespace gnc {

struct BodyWrench {
	double Fz = 0.0;
	double Mx = 0.0;
	double My = 0.0;
	double Mz = 0.0;
};

struct MotorCmd {
	Vec4 u;
	MotorCmd() {
		u.setZero();
	}
};

}