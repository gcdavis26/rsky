#include "estimation/AHRS.h"

AHRS::AHRS()
	: AHRSbias(Vec<6>::Zero()),
	phi(0.0),theta(0.0),psi(0.0)
{
	x.setZero();
	P.setIdentity();
	P(0, 0) = 1e-2; // phi
	P(1, 1) = 1e-2; // theta
	P(2, 2) = 1e-3; // bgx
	P(3, 3) = 1e-3; // bgy
}

AHRS::AHRS(const Vec<6>& bias) 
	: phi(0.0),theta(0.0),psi(0.0)
{
	AHRSbias = bias;
	x.setZero();
	P.setIdentity();
	P(0, 0) = 1e-2; // phi
	P(1, 1) = 1e-2; // theta
	P(2, 2) = 1e-3; // bgx
	P(3, 3) = 1e-3; // bgy
}

void AHRS::initializeFromAccel(const Vec<3>& accel) {
	Vec<3> AHRSaccel = accel - AHRSbias.segment<3>(3);
	double r = 0.0, p = 0.0;
	accelToAttitude(AHRSaccel, r, p);
	initialize(r, p);
}

void AHRS::initialize(double roll, double pitch) {
	phi = roll;
	theta = clamp(pitch, -max_abs_pitch_rad, max_abs_pitch_rad);

	// EKF state sync
	x(0) = phi;
	x(1) = theta;
	x(2) = 0.0;
	x(3) = 0.0;
}

void AHRS::update(const Vec<3>& accel,const Vec<3>& gyro, double dt) {
	if (!(std::isfinite(dt) && dt > 0.0)) return;
	
	//apply bias
	Vec<3> AHRSgyro = gyro - AHRSbias.segment<3>(0);
	Vec<3> AHRSaccel = accel - AHRSbias.segment<3>(3);

	// accel gate
	const double amag = AHRSaccel.norm();
	const bool accel_ok = std::isfinite(amag) && std::abs(amag - g) <= (gate_g * g);

	// ekf predict 
	double ph = x(0);
	double th = x(1);
	double bgx = x(2);
	double bgy = x(3);

	// bias correct
	const double p = AHRSgyro(0) - bgx;
	const double q = AHRSgyro(1) - bgy;
	const double r = AHRSgyro(2);

	// avoid tan/sec blowup
	th = clamp(th, -max_abs_pitch_rad, max_abs_pitch_rad);

	const double sphi = std::sin(ph), cphi = std::cos(ph);
	const double sth = std::sin(th), cth = std::cos(th);
	const double tth = std::tan(th);
	const double sec2 = 1.0 / (cth * cth + 1e-12); // sec^2(theta), guarded

	// Continuous-time dynamics 
	const double ph_dot = p + sphi * tth * q + cphi * tth * r;
	const double th_dot = cphi * q - sphi * r;

	ph += dt * ph_dot;
	th += dt * th_dot;
	th = clamp(th, -max_abs_pitch_rad, max_abs_pitch_rad);

	x(0) = ph;
	x(1) = th;

	// jacobian
	Mat<4,4> F = Mat<4,4>::Zero();

	// d(ph_dot)/dphi, d(ph_dot)/dtheta
	const double dphdphi = (cphi * tth) * q + (-sphi * tth) * r;
	const double dphdtheta = (sphi * sec2) * q + (cphi * sec2) * r;

	// d(th_dot)/dphi
	const double dthdphi = (-sphi) * q + (-cphi) * r;

	// Bias derivatives
	const double dph_dbgx = -1.0;
	const double dph_dbgy = -(sphi * tth);
	const double dth_dbgx = 0.0;
	const double dth_dbgy = -(cphi);

	F(0, 0) = dphdphi;
	F(0, 1) = dphdtheta;
	F(0, 2) = dph_dbgx;
	F(0, 3) = dph_dbgy;

	F(1, 0) = dthdphi;
	F(1, 1) = 0.0;
	F(1, 2) = dth_dbgx;
	F(1, 3) = dth_dbgy;

	Mat<4,4> Phi = Mat<4,4>::Identity() + F * dt;

	Mat<4,4> Q = Mat<4,4>::Zero();
	Q(0, 0) = q_angle * dt;
	Q(1, 1) = q_angle * dt;
	Q(2, 2) = q_bias * dt;
	Q(3, 3) = q_bias * dt;

	P = Phi * P * Phi.transpose() + Q;

	// ekf update
	if (accel_ok) {
		double phi_acc = 0.0, theta_acc = 0.0;
		accelToAttitude(AHRSaccel, phi_acc, theta_acc);

		Vec<2> z;
		z << phi_acc, theta_acc;

		Vec<2> h;
		h << x(0), x(1);
		Mat<2, 4> H = Mat<2, 4>::Zero();

		H(0, 0) = 1.0;
		H(1, 1) = 1.0;

		Mat<2, 2> R = Mat<2, 2>::Zero();
		R(0, 0) = r_tilt;
		R(1, 1) = r_tilt;

		Mat <2, 2> S = H * P * H.transpose() + R;
		Mat<4, 2> K = P * H.transpose() * S.inverse();

		x = x + K * (z - h);

		Mat<4, 4> I = Mat <4, 4>::Identity();
		Mat<4, 4> IKH = I - K * H;
		P = IKH * P * IKH.transpose() + K * R * K.transpose();

		x(1) = clamp(x(1), -max_abs_pitch_rad, max_abs_pitch_rad);
	}

	phi = x(0);
	theta = x(1);

	Vec<3> gyro_for_psi;
	gyro_for_psi << (AHRSgyro(0) - x(2)), (AHRSgyro(1) - x(3)), AHRSgyro(2);

	Vec<3> eulerRates = eulerRates_ZYX(phi, theta, gyro_for_psi);
	psi = wrapToPi(psi + dt * eulerRates(2));
}

//already bias corrected before getting passed into this
void AHRS::accelToAttitude(const Vec<3>& accel, double& roll, double& pitch) {
	const double ax = accel(0);
	const double ay = accel(1);
	const double az = accel(2);

	roll = std::atan2(-ay, -az);
	pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
}