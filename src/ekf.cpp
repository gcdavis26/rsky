#include <Eigen/Dense>
#include "ekf.h"
#include "math_utils.h"


EKF::EKF(Eigen::Vector3d r, Eigen::Matrix<double, 15, 1> x0, Eigen::Vector3d gyro0, Eigen::Vector3d accel0,Eigen::Matrix<double, 12, 1> sigmaw, Eigen::Matrix<double, 4, 1> sigmav, double freq)
{
	g = Eigen::Vector3d::Zero();
	g(2) = 9.81; //setting up gravity
	alpha = Eigen::Vector3d::Zero();
	omega_measured = gyro0;
	body_accels = accel0;
	dt = 1 / freq;
	x = x0; //phi, theta, psi, n, e, d, vn,ve,vd, b_ax,b_ay,b_az, b_p,b_q,b_r
	Q = sigmaw.asDiagonal() * sigmaw.asDiagonal();
	R = sigmav.asDiagonal() * sigmav.asDiagonal();
	P = Eigen::Matrix<double, 15, 15>::Zero();
	radius = r;
}

void EKF::estimate(Eigen::Vector3d omega, Eigen::Vector3d new_body_accels) 
{
	
	//despite how this looks, the omega and body_accels collected here are not used in the state estimation
	//at the time collected. The priori information from the previous timestep are used. Derivatives at time t 
	//are not useful for calculating state at time t.
	//correct body_accels to be acting on CG. Rigid body means that omega isn't impacted
	Eigen::Vector3d measured_inertial_accels =  body_accels + alpha.cross(radius) + omega_measured.cross(omega_measured.cross(radius)); 
	//everything here uses the prior omega, body_accels from previous timestep. 
	Eigen::Matrix<double,15,1> xdot = get_xdot(x, g, measured_inertial_accels, omega_measured);
	A = jacobian(x, g, measured_inertial_accels, omega_measured);
	Eigen::Matrix<double, 15, 12> G = noise_coupling(x);
	Eigen::Matrix<double, 15, 15>pdot1 = A * P + P * A.transpose() + G*Q*G.transpose();
	Eigen::Matrix<double, 15, 15>pdot2 = A * (P + pdot1 * dt/2) + (P + pdot1 * dt / 2) * A.transpose() + G * Q * G.transpose();
	x = x + get_xdot(x + dt / 2 * get_xdot(x, g, measured_inertial_accels, omega_measured), g, measured_inertial_accels, omega_measured) * dt; //euler integration
	P = P + pdot2 * dt; //euler integration
	P = (P + P.transpose()) / 2.0; //enforcing symmetry

	Eigen::Vector3d alpha_raw = (omega - omega_measured) / dt;//updating our rates and accelerations for the next prediction 
	alpha = alpha * .7 + (1.0 - .7) * alpha_raw;
	omega_measured = omega;
	body_accels = new_body_accels;
}

void EKF::update(Eigen::Matrix<double, 4, 1> m)
{
	Eigen::Matrix<double, 4, 15> H = Eigen::Matrix<double, 4, 15>::Zero();
	H.block(0, 2, 4, 4) = Eigen::Matrix<double, 4, 4>::Identity();
	Eigen::Matrix<double, 4, 1> res = m - H * x;
	Eigen::Matrix<double, 15, 4> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

	x = x + K * res;
	P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P * (Eigen::Matrix<double, 15, 15>::Identity() - K * H).transpose() + K * R * K.transpose();
	P = (P + P.transpose()) / 2.0;
	x(0) = wrapPi(x(0));
	x(1) = wrapPi(x(1));
	x(2) = wrapPi(x(2));
}

Eigen::Matrix<double, 15, 1> EKF::getState()
{
	return x;
}

Eigen::Vector3d EKF::getOmega()
{
	return omega_measured;
}