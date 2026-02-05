#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include "math_utils.h"
#include <iostream>
using namespace std;
Matrix3d dcmI_B(double phi, double theta, double psi)
{
	//This returns a DCM that goes from body frame to inertial
    Matrix3d Z_rot;
	Matrix3d Y_rot;
	Matrix3d X_rot;

	Z_rot << cos(psi), -sin(psi), 0,
		sin(psi), cos(psi), 0,
		0, 0, 1;
	Y_rot << cos(theta), 0, sin(theta),
		0, 1, 0,
		-sin(theta), 0, cos(theta);
	X_rot << 1, 0, 0,
		0, cos(phi), -sin(phi),
		0, sin(phi), cos(phi);

	return Z_rot * Y_rot * X_rot;
}

////////////////////////////
////EKF FUNCTIONS

Vector15d get_xdot(Vector15d x, Vector3d g, Vector3d a_body_measured, Vector3d omega)
{
	//note that a_measured is the IMU measurement, as measured from the body frame. No gravity included, and still in body vectors.
	//a_inertial_measured includes the correction due to off-cg IMU location, so it is a CG acceleration
	//x is expected to be attitude, pos, v, accel bias, gyro bias
	Vector15d xdot;
	Matrix3d T; //This is the transform from body rates to euler rates
	T << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));
	xdot.block(0, 0, 3, 1) = T * omega;
	xdot.block(3, 0, 3, 1) = x.block(6, 0, 3, 1);
	xdot.block(6, 0, 3, 1) = dcmI_B(x(0), x(1), x(2)) * a_body_measured + g;
	xdot.block(9,0,6,1) = Eigen::Matrix<double, 6, 1>::Zero();
	return xdot;
}

Matrix15d jacobian(Vector15d x, Vector3d g, Vector3d a_body_measured, Vector3d omega)
{ 
	double eps = 1e-6;
	Matrix15d A;
	for (int i = 0; i < 15; i++)
	{
		Vector15d dx;
		dx.setZero();
		dx(i) = eps;
		A.col(i) = (get_xdot(x + dx, g, a_body_measured, omega) - get_xdot(x - dx, g, a_body_measured, omega)) / (2 * eps);
	}
	return A;
}
Eigen::Matrix<double, 15, 12> noise_coupling(Vector15d x)
{
	Matrix3d T;
	T << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));

	Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();
	G.block(0, 0, 3, 3) = T;
	G.block(6, 3, 3, 3) = dcmI_B(x(0), x(1), x(2));
	G.block(9, 6, 3, 3) = Matrix3d::Identity();
	G.block(12, 9, 3, 3) = Matrix3d::Identity();
	
	return G;
}

//////////////////
//Simulator functions

Vector3d sim_imu_accels(Vector12d x_true, Vector3d commanded_body_accel, Vector3d alpha, Vector3d r, Vector3d imunoise)
{
	//x_true is attitude, body rates, pos, v
	// commanded_body_accels is true non-gravity acceleration in the body frame, at the COM of the object. Essentially, for quadrotor, specific thrust. 
	// r = IMU - CG (body frame). 
	// In EKF, we use -r to map IMU measurement back to CG.	
	//returns IMU acceleration reading for a given body acceleration at CG. 

	Vector3d g;
	g << 0, 0, 9.81;
	Vector3d omega_measured = x_true.block(3, 0, 3, 1);
	Vector3d off_cg_accels = commanded_body_accel + alpha.cross(r) + omega_measured.cross(omega_measured.cross(r)); //moving imu stuff to COM
	
	double ground = 0.0;
	double penetration = x_true(8) - ground; // negative if above ground
	if (penetration > 0) 
	{  // below ground and moving down
		
		double k_ground = 1000; // spring-like constant
		double b_ground = 50;   // damping
		Vector3d off_cg_inertial = dcmI_B(x_true(0), x_true(1), x_true(2)) * off_cg_accels;
		off_cg_inertial(2) += -k_ground * penetration - b_ground * x_true(11);
		off_cg_accels = dcmI_B(x_true(0), x_true(1), x_true(2)).transpose() * off_cg_inertial; //gravity not subtracted because it is never added in the first place
	}
	
	Vector3d a_measured = off_cg_accels + imunoise;

	return a_measured;
}
Vector3d sim_gyro_rates(Vector12d x_true, Vector3d gyronoise)
{
	return x_true.block(3, 0, 3, 1) + gyronoise;
}

Vector3d sim_measurement(Vector3d x_true_measured, Vector3d m_noise)
{
	return x_true_measured + m_noise;
}

//GET DYNAMICS DOES NOT USE THE SAME STATES AS EKF. get_xdot is for the KALMAN FILTER. This is for propagating true 12d state: attitude, body rates, pos, vel
Vector12d get_dynamics(Vector12d x, Vector3d g, double m, Vector3d inertias, double thrust, Vector3d moments)
{
	//x = phi theta psi, p q r, n e d, vn ve vd, 
	//xdot = euler_rates, omegadot, vn ve vd, an ae ad, 
	double L = moments(0);
	double M = moments(1);
	double N = moments(2);
	double Ix = inertias(0);
	double Iy = inertias(1);
	double Iz = inertias(2);
	
	Matrix3d to_euler; //This is the transform from body rates to euler rates
	to_euler << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));
	Vector12d xdot;
	Vector3d forces;
	forces << 0, 0, -thrust;
	xdot.block(0, 0, 3, 1) = to_euler * x.block(3, 0, 3, 1);
	xdot(3) = ((Iy - Iz) * x(4) * x(5) + L) / Ix;
	xdot(4) = ((Iz - Ix) * x(3) * x(5) + M) / Iy;
	xdot(5) = ((Ix - Iy) * x(3) * x(4) + N) / Iz;
	xdot.block(6, 0, 3, 1) = x.block(9, 0, 3, 1);
	xdot.block(9, 0, 3, 1) = dcmI_B(x(0), x(1), x(2)) * forces / m + g;


	
	double penetration = x(8); // negative if above ground
	if (penetration > 0)
	{  // below ground and moving down
		double k_ground = 1000; // spring-like constant
		double b_ground = 50;   // damping
		xdot(11) += -k_ground * penetration - b_ground * x(11);
	}
	
	return xdot;
}


//////////////////
//Utility functions

Vector12d noise12d()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dist(0.0, 1.0);
	Vector12d noise;
	for (int i = 0; i < 12; i++)
	{
		noise(i) = dist(gen);
	}
	return noise;
}

Vector3d noise3d()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dist(0.0, 1.0);
	Vector3d noise;
	for (int i = 0; i < 3; i++)
	{
		noise(i) = dist(gen);
	}
	return noise;
}

double wrapPi(double angle)
{
	return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

double saturate(double command, double saturation, bool ismax)
{
	if (ismax == true)
	{
		if (std::abs(command) > saturation)
		{
			return std::copysign(saturation, command);
		}
		else
		{
			return command;
		}
	}
	else //saturation minimum
	{
		if (std::abs(command) < saturation)
		{
			return std::copysign(saturation, command);
		}
		else
		{
			return command;
		}
	}
	
}

double clamp(double x, double lo, double hi) {
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}