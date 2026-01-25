#include <Eigen/Dense>
#include "ekf.h"
#include "math_utils.h"
#include <iostream>

EKF::EKF(Eigen::Vector3d r, Eigen::Matrix<double, 15, 1> x0, Eigen::Vector3d gyro0, Eigen::Vector3d accel0, Eigen::Matrix<double, 12, 1> sigmaw, Eigen::Vector3d sigmav, double freq)
{
	//r = IMU - CG position = IMU pos (CG = 0,0,0).
	//constructor, setting up important vectors, matrices
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

void EKF::estimate(Eigen::Vector3d omega, Eigen::Vector3d new_imu_accels) 
{
	
	//despite how this looks, the omega and body_accels collected here are not used in the state estimation
	//at the time collected. The priori information from the previous timestep are used. Derivatives at time t 
	//are not useful for calculating state at time t.
	//correct body_accels to be acting on CG. Rigid body means that omega isn't impacted
	Eigen::Vector3d com_body_accels =  body_accels + alpha.cross(-radius) + omega_measured.cross(omega_measured.cross(-radius)); //moving imu stuff to COM
	//everything here uses the prior omega, body_accels from previous timestep. 
	Eigen::Matrix<double,15,1> xdot = get_xdot(x, g, com_body_accels, omega_measured);
	A = jacobian(x, g, com_body_accels, omega_measured);
	Eigen::Matrix<double, 15, 12> G = noise_coupling(x); 
	Eigen::Matrix<double, 15, 15>pdot = A * P + P * A.transpose() + G*Q*G.transpose();
	x = x + get_xdot(x, g, com_body_accels, omega_measured) * dt; //euler integration
	P = P + pdot * dt; //euler integration
	P = (P + P.transpose()) / 2.0; //enforcing symmetry 
	//std::cout <<"Velocity derivative: " << std::endl << get_xdot(x, g, com_body_accels, omega_measured).block(6, 0, 3, 1) << std::endl << "Body accels: " << std::endl << com_body_accels << std::endl;

	Eigen::Vector3d alpha_raw = (omega - omega_measured) / dt;//updating our rates and accelerations for the next prediction 
	alpha = alpha * .7 + (1.0 - .7) * alpha_raw;
	alpha = alpha_raw; //for testing
	omega_measured = omega; //updating the process model measurements for the next step
	body_accels = new_imu_accels;
}

void EKF::update(Eigen::Vector3d m)
{
	//incorporate a measurement model measurement into state estimate
	Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
	H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
	Eigen::Vector3d res = m - H * x; //calculating residual
	Eigen::Matrix<double, 15, 3> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

	x = x + K * res; //incorporating residual via kalman gain
	P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P * (Eigen::Matrix<double, 15, 15>::Identity() - K * H).transpose() + K * R * K.transpose();
	P = (P + P.transpose()) / 2.0;
	x(0) = wrapPi(x(0)); //ensuring angles are staying within -pi to pi
	x(1) = wrapPi(x(1));
	x(2) = wrapPi(x(2));
}

Eigen::Matrix<double, 15, 1> EKF::getState()
{
	return x;
}

Eigen::Matrix<double, 12, 1> EKF::getControlState()
{
	Eigen::Matrix<double, 12, 1> controlState;
	controlState.block(0, 0, 3, 1) = x.block(0,0,3,1);
	controlState.block(3, 0, 3, 1) = omega_measured;
	controlState.block(6, 0, 6, 1) = x.block(3, 0, 6, 1);
	return controlState;
}

Eigen::Vector3d EKF::getOmega()
{
	return omega_measured;
}