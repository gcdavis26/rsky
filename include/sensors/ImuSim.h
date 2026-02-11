#pragma once
#include "common/MathUtils.h"

class ImuSim {
public:
	struct ImuMeas {
		Vec<3> gyro = Vec<3>::Zero();
		Vec<3> accel = Vec<3>::Zero();
	};

	ImuMeas imu;

	ImuSim();

	void step(const Vec<12>& truth, double dt);

private:
	bool has_prev = false;
	Vec<3> v_prev = Vec<3>::Zero();
	double gyro_noise_std = 0.01;
	double accel_noise_std = 0.01;

	Vec<3> gyro_bias = Vec<3>::Zero();
	Vec<3> accel_bias = Vec<3>::Zero();

	std::mt19937 rng;
	std::normal_distribution<double> norm;

};