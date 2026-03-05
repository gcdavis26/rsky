#pragma once

#include "common/MathUtils.h"

class AHRS {
public:
    AHRS();
	AHRS(const Vec<6>& bias);

	void initializeFromAccel(const Vec<3>& accel); //optional add yaw init later
	void initialize(double roll, double pitch);
	void update(const Vec<3>& accel,const Vec<3>& gyro, double dt);

	Vec<3> euler() const { return Vec<3>(phi, theta, psi); }

	bool init = false;

private:
    void accelToAttitude(const Vec<3>& accel, double& roll, double& pitch);

    // Existing
    Vec<6> AHRSbias;
    double phi = 0.0, theta = 0.0, psi = 0.0;

    // ==== 4-state EKF: x = [phi, theta, bgx, bgy] ====
    Vec<4> x = (Vec<4>() << 0, 0, 0, 0).finished();
    Mat<4,4> P = Mat<4,4>::Identity();

    // Tunables (set these somewhere sensible for your IMU)
    double q_angle = 2e-5;     // process noise for angles [rad^2/s]  (discrete uses *dt)
    double q_bias = 0.0;     // bias random-walk noise [ (rad/s)^2 / s ] (discrete uses *dt)
    double r_tilt = 5e-4;     // accel tilt-meas noise [rad^2] (applied to phi_acc,theta_acc)

    // Existing gates/constants (assumed you have these)
    double gate_g = 0.15;      // e.g. 15% gate
    double max_abs_pitch_rad = 1.50; // example
};