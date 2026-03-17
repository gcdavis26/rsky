#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "common/MathUtils.h"

class AHRS {
public:
	AHRS();
	explicit AHRS(const Vec<6>& bias);

	// Initialization
	void initializeFromAccel(const Vec<3>& accel);
	void initialize(double roll, double pitch);

	// Main update
	void update(const Vec<3>& accel, const Vec<3>& gyro, double dt);

	// Accessors
	Vec<3> euler() const {
		Vec<3> e;
		e << phi, theta, psi;
		return e;
	}

	bool init = false;

private:
	// --- State ---
	double phi;   // roll
	double theta; // pitch
	double psi;   // yaw

	Vec<3> bg; // estimated gyro bias

	// Optional static sensor biases (gyro[0:2], accel[3:5])
	Vec<6> AHRSbias;

	// --- Parameters / tuning ---
	double kp_acc = 3.0;   // proportional accel correction
	double ki_acc = 0.05;  // integral accel correction (bias learning)

	double gate_g = 0.15;  // accel magnitude gate (fraction of g)
	double g = 9.81;       // gravity magnitude

	double max_abs_pitch_rad = 1.553343; // ~89 deg

	// --- Helpers ---
	void accelToAttitude(const Vec<3>& accel, double& roll, double& pitch);
};