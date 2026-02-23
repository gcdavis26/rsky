#include <Eigen/Dense>
#include "ekf.h"
#include "math_utils.h"
#include <iostream>

EKF::EKF(const Vector3d& r, const Vector12d& sigmaw, const Vector3d& sigmav)
{
	//r = IMU - CG position = IMU pos (CG = 0,0,0).
	//constructor, setting up important vectors, matrices
	g.setZero();
	g(2) = 9.81; //setting up gravity
	alpha.setZero();
	omega_measured.setZero();
	body_accels.setZero();
	x.setZero(); //phi, theta, psi, n, e, d, vn,ve,vd, b_ax,b_ay,b_az, b_p,b_q,b_r
	Q = sigmaw.array().square().matrix().asDiagonal();
	R = sigmav.array().square().matrix().asDiagonal();
	P.setZero();
	radius = r;
	G.setZero();
	K.setZero();

}

void EKF::initialize(const Vector3d& measurement, const Vector3d& gyro0, const Vector3d& accel0, const Vector3d& bias_accel, const Vector3d& bias_gyro) //set up initial states. Initial measurement, per se
{
	omega_measured = gyro0;
	body_accels = accel0;
	x.block(3, 0, 3, 1) = measurement;
	x.block(9, 0, 3, 1) = bias_accel;
	x.block(12, 0, 3, 1) = bias_gyro;
}
void EKF::imureading(const Vector3d& omega, const Vector3d& new_imu_accels, double dt)
{
	Vector3d alpha_raw;
	alpha_raw = (omega - omega_measured) / dt;//updating our rates and accelerations for the next prediction 
	alpha = alpha * .7 + (1.0 - .7) * alpha_raw;
	//alpha = alpha_raw;  testing
	omega_measured = omega; //updating the process model measurements for the next step
	body_accels = new_imu_accels;
}

void EKF::estimate(double dt) 
{
	//correct body_accels to be acting on CG. Rigid body means that omega isn't impacted
	Vector3d com_body_accels = body_accels + alpha.cross(-radius) + omega_measured.cross(omega_measured.cross(-radius)); //moving imu stuff to COM
	//everything here uses the prior omega, body_accels from previous timestep. 
	Vector15d xdot = get_xdot(x, g, com_body_accels, omega_measured);
	A = jacobian(x, g, com_body_accels, omega_measured);
	G = noise_coupling(x);
	x = x + xdot * dt; //euler integration
	Matrix15d pdot;
	Matrix15d AP;
	AP.noalias() = A * P;
	pdot.noalias() = AP + AP.transpose() + G * Q * G.transpose();
	P = P + pdot * dt; //euler integration
	//std::cout <<"Velocity derivative: " << std::endl << get_xdot(x, g, com_body_accels, omega_measured).block(6, 0, 3, 1) << std::endl << "Body accels: " << std::endl << com_body_accels << std::endl;
}

void EKF::update(const Vector3d& m)
{

	//incorporate a measurement model measurement into state estimate
	Vector3d res;
	res.noalias()  = m - x.block(3,0,3,1); //calculating residual
	Matrix3d S;
	S = P.block(3, 3, 3, 3).selfadjointView<Eigen::Lower>();
	S += R;
	K.noalias() = P.block(0,3,15,3) * (S).ldlt().solve(Matrix3d::Identity());;
	//Matrix15d KH;
	//KH.noalias() = K * H;
	x = x + K * res; //incorporating residual via kalman gain
	//P = (I15d - KH) * P * (I15d - KH).transpose() + K * R * K.transpose(); //Joseph stable form
	P.noalias() -= K * S * K.transpose();
	P = P.selfadjointView<Eigen::Lower>();//symmetry
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