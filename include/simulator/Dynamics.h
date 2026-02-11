#pragma once

#include <random>
#include "common/MathUtils.h"

class Dynamics {
public:
	Dynamics();
	void step(double dt, const Vec<4>& bodyWrench);
	Vec<12> getTrueState();

private:
	double Ix = 0.01;
	double Iy = 0.01;
	double Iz = 0.02;

	Vec<3> sigma_turb = Vec<3>::Zero();

	std::mt19937 rng;
	std::normal_distribution<double> norm;

	Vec<12> state = Vec<12>::Zero(); // pos vel euler omega
};

