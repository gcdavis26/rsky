#pragma once
#include "common/MathUtils.h"

class OptiSim {
public:
	struct OptiMeas {
		Vec<3> pos = Vec<3>::Zero();
		double psi = 0.0;
	};
	OptiMeas opti;

	OptiSim();

	void step(const Vec<12>& truth);

private:
	double pos_noise_std = 0.001;
	double psi_noise_std = 0.001;

	std::mt19937 rng;
	std::normal_distribution<double> norm;
};