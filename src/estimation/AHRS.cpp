#include "estimation/AHRS.h"

// Mahony-style Euler AHRS (roll/pitch from accel, yaw from integrating gyro)
// State: phi, theta, psi, and gyro bias b_g (estimated)
// Uses accel magnitude gate: only correct when |a| ≈ g

AHRS::AHRS()
	: AHRSbias(Vec<6>::Zero()),
	phi(0.0), theta(0.0), psi(0.0),
	bg(Vec<3>::Zero())
{
}

AHRS::AHRS(const Vec<6>& bias)
	: phi(0.0), theta(0.0), psi(0.0),
	bg(Vec<3>::Zero())
{
	AHRSbias = bias;
}

void AHRS::initializeFromAccel(const Vec<3>& accel) {
	Vec<3> a = accel - AHRSbias.segment<3>(3); // subtract accel bias if you have one
	double r = 0.0, p = 0.0;
	accelToAttitude(a, r, p);
	initialize(r, p);
}

void AHRS::initialize(double roll, double pitch) {
	phi = roll;
	theta = clamp(pitch, -max_abs_pitch_rad, max_abs_pitch_rad);
	// keep psi as-is (or set to 0 if you want)
}

void AHRS::update(const Vec<3>& accel, const Vec<3>& gyro, double dt) {
	if (!(std::isfinite(dt) && dt > 0.0)) return;

	// accel (bias-correct if you have a static bias estimate)
	const Vec<3> a = accel - AHRSbias.segment<3>(3);

	// accel gate: only trust accel when magnitude ~ g
	const double amag = a.norm();
	const bool accel_ok = std::isfinite(amag) && std::abs(amag - g) <= (gate_g * g);

	// Mahony correction term
	Vec<3> e = Vec<3>::Zero();

	if (accel_ok && amag > 1e-6) {
		const Vec<3> a_hat = a / amag; // measured "gravity direction" (actually specific force)

		// predicted accel direction from current (phi,theta) under gravity-only:
		// a_pred = [ g*sin(theta), -g*sin(phi)*cos(theta), -g*cos(phi)*cos(theta) ]
		// unit direction:
		const double sphi = std::sin(phi), cphi = std::cos(phi);
		const double sth = std::sin(theta), cth = std::cos(theta);

		Vec<3> v_hat;
		v_hat << sth, -sphi * cth, -cphi * cth; // already unit-length (numerically close)

		// error to rotate estimate toward measurement (Mahony: e = a_hat x v_hat)
		e = a_hat.cross(v_hat);
	}

	// PI correction on gyro (bias estimate + proportional feedback)
	// NOTE: tune kp_acc, ki_acc in your AHRS.h (typical: kp ~ 1..10, ki ~ 0..1)
	bg += (ki_acc * e) * dt;

	Vec<3> omega_corr = gyro - bg + (kp_acc * e);

	// Integrate Euler angles using your existing helper
	theta = clamp(theta, -max_abs_pitch_rad, max_abs_pitch_rad);

	Vec<3> eRates = eulerRates_ZYX(phi, theta, omega_corr);

	phi = wrapToPi(phi + dt * eRates(0));
	theta = clamp(theta + dt * eRates(1), -max_abs_pitch_rad, max_abs_pitch_rad);
	psi = wrapToPi(psi + dt * eRates(2));
}

// already bias corrected before getting passed into this
void AHRS::accelToAttitude(const Vec<3>& accel, double& roll, double& pitch) {
	const double ax = accel(0);
	const double ay = accel(1);
	const double az = accel(2);

	roll = std::atan2(-ay, -az);
	pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
}