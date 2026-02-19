#include "estimation/AHRS.h"

void AHRS::initializeFromAccel(Vec<3> accel) {
	double r = 0.0, p = 0.0;
	accelToAttitude(accel, r, p);
	initialize(r, p);
}

void AHRS::initialize(double roll, double pitch) {
	phi = roll;
	theta = clamp(pitch, -max_abs_pitch_rad, max_abs_pitch_rad);
}

void AHRS::update(const Vec<3>& accel, const Vec<3>& gyro, double dt) {
	if (!(std::isfinite(dt) && dt > 0.0)) return;

	// gyro propagation
	Vec<3> eulerRates1 = eulerRates_ZYX(phi, theta, gyro);


	const double phi_g = phi + eulerRates1(0) * dt;
	const double theta_g = theta + eulerRates1(1) * dt;

	//Vec<3> eulerRates2 = eulerRates_ZYX(phi_g1, theta_g1, gyro);

	//const double phi_g = phi + dt / 2 * (eulerRates1(0) + eulerRates2(0));
	//const double theta_g = theta + dt / 2 * (eulerRates1(1) + eulerRates2(1));
	const double psi_g = psi + dt * (eulerRates1(2)); // + eulerRates2(2));


	// accel tilt measurement
	double phi_acc = 0.0, theta_acc = 0.0;
	accelToAttitude(accel, phi_acc, theta_acc);

	// filter gain
	const double tau = std::max(1e-6, tau);
	const double alpha = tau / (tau + dt);

	// gate accel correction when |a| is not near g
	const double amag = accel.norm();
	const bool accel_ok = std::isfinite(amag) && std::abs(amag - g) <= (gate_g * g);

	if (accel_ok) {
		phi = alpha * phi_g + (1 - alpha) * phi_acc;
		theta = alpha * theta_g + (1 - alpha) * theta_acc;
	}
	else {
		phi = phi_g;
		theta = theta_g;
	}

	psi = wrapToPi(psi_g);

}

void AHRS::accelToAttitude(const Vec<3>& accel, double& roll, double& pitch) {
	const double ax = accel(0);
	const double ay = accel(1);
	const double az = accel(2);

	roll = std::atan2(-ay, -az);
	pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
}