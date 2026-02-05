#include <Eigen/Dense>
#include "ekf.h"
#include "math_utils.h"
#include <iostream>

EKF::EKF(Vector3d r, Vector12d sigmaw, Vector3d sigmav, double freq)
{
	//r = IMU - CG position = IMU pos (CG = 0,0,0).
	//constructor, setting up important vectors, matrices
	g.setZero();
	g(2) = 9.81; //setting up gravity
	alpha.setZero();
	omega_measured.setZero();
	body_accels.setZero();
	dt = 1 / freq;
	x.setZero(); //phi, theta, psi, n, e, d, vn,ve,vd, b_ax,b_ay,b_az, b_p,b_q,b_r
	Q = sigmaw.asDiagonal() * sigmaw.asDiagonal();
	R = sigmav.asDiagonal() * sigmav.asDiagonal();
	P = Matrix15d::Zero();
	radius = r;
}

void EKF::initialize(Vector3d measurement, Vector3d gyro0, Vector3d accel0, Vector3d bias_accel, Vector3d bias_gyro) //set up initial states. Initial measurement, per se
{
	omega_measured = gyro0;
	body_accels = accel0;
	x.block(3, 0, 3, 1) = measurement;
	x.block(9, 0, 3, 1) = bias_accel;
	x.block(12, 0, 3, 1) = bias_gyro;
}
void EKF::imureading(Vector3d omega, Vector3d new_imu_accels)
{
	Vector3d alpha_raw = (omega - omega_measured) / dt;//updating our rates and accelerations for the next prediction 
	alpha = alpha * .7 + (1.0 - .7) * alpha_raw;
	alpha = alpha_raw; //for testing
	omega_measured = omega - x.block(12,0,3,1); //updating the process model measurements for the next step
	body_accels = new_imu_accels - x.block(9, 0, 3, 1);
}

void EKF::estimate() 
{
	//correct body_accels to be acting on CG. Rigid body means that omega isn't impacted
	Vector3d com_body_accels =  body_accels + alpha.cross(-radius) + omega_measured.cross(omega_measured.cross(-radius)); //moving imu stuff to COM
	//everything here uses the prior omega, body_accels from previous timestep. 
	Vector15d xdot = get_xdot(x, g, com_body_accels, omega_measured);
	A = jacobian(x, g, com_body_accels, omega_measured);
	Eigen::Matrix<double, 15, 12> G = noise_coupling(x); 
	Matrix15d pdot = A * P + P * A.transpose() + G*Q*G.transpose();
	x = x + get_xdot(x, g, com_body_accels, omega_measured) * dt; //euler integration
	P = P + pdot * dt; //euler integration
	P = (P + P.transpose()) / 2.0; //enforcing symmetry 
	//std::cout <<"Velocity derivative: " << std::endl << get_xdot(x, g, com_body_accels, omega_measured).block(6, 0, 3, 1) << std::endl << "Body accels: " << std::endl << com_body_accels << std::endl;
}

void EKF::update(Vector3d m)
{
	//incorporate a measurement model measurement into state estimate
	Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
	H.block(0, 3, 3, 3) = Matrix3d::Identity();
	Vector3d res = m - H * x; //calculating residual
	Eigen::Matrix<double, 15, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

	x = x + K * res; //incorporating residual via kalman gain
	P = (Matrix15d::Identity() - K * H) * P * (Matrix15d::Identity() - K * H).transpose() + K * R * K.transpose();
	P = (P + P.transpose()) / 2.0;
	x(0) = wrapPi(x(0)); //ensuring angles are staying within -pi to pi
	x(1) = wrapPi(x(1));
	x(2) = wrapPi(x(2));
}

Vector15d EKF::getState()
{
	return x;
}

Vector12d EKF::getControlState()
{
	Vector12d controlState;
	controlState.block(0, 0, 3, 1) = x.block(0,0,3,1);
	controlState.block(3, 0, 3, 1) = omega_measured;
	controlState.block(6, 0, 6, 1) = x.block(3, 0, 6, 1);
	return controlState;
}

Vector3d EKF::getOmega()
{
	return omega_measured;
}