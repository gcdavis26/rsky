#pragma once
#include "common/types.h"

namespace gnc {

struct MixerParams {
	double arm_length = 0.25;
	double k_yaw = 0.02;
	double T_min = 0.0;
	double T_max = 2 * MASS * GRAVITY / 4.0;
};

}