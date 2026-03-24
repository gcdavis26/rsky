#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include "common/MathUtils.h"

class AHRS {
public:
	AHRS();
	explicit AHRS(const Vec<6>& bias);

	// Initialization
	void initializeFromAccel(const Vec<3>& accel);
	void initialize(double roll, double pitch, double yaw = 0.0);

	// Main update
	void update(const Vec<3>& accel, const Vec<3>& gyro, double dt);

	// Accessors
	Vec<3> euler() const;

	bool init = false;

private:
	// --- State ---
	Eigen::Quaterniond q;  // body-to-NED quaternion (replaces phi/theta/psi)
	Vec<3> bg;             // estimated gyro bias
	Vec<6> AHRSbias;       // static sensor biases (gyro[0:2], accel[3:5])

	// --- Parameters / tuning ---

	double kp_acc = 3.0;
	double ki_acc = 0.05;

	double gate_g = 0.15;
	double max_abs_pitch_rad = 1.553343;

	// --- Helpers ---
	void accelToAttitude(const Vec<3>& accel, double& roll, double& pitch);
};
