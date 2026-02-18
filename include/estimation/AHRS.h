#pragma once

#include "common/MathUtils.h"

class AHRS {
public:

	void initializeFromAccel(Vec<3> accel); //optional add yaw init later

	void initialize(double roll, double pitch);

	void update(const Vec<3>& accel, const Vec<3>& gyro, double dt);

	Vec<3> euler() const { return Vec<3>(phi, theta, psi); }

	bool init = false;

private:
	double tau = 0.5;
	double gate_g = 0.30;
	double max_abs_pitch_rad = 80 * PI / 180;

	double phi = 0.0;
	double theta = 0.0;
	double psi = 0.0;

	void accelToAttitude(const Vec<3>& accel, double& roll, double& pitch);
};