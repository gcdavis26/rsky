#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include "math_utils.h"
#include <iostream>
using namespace std;
Matrix3d dcmI_B(double phi, double theta, double psi)
{
	Matrix3d dcm;
	double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi = cos(psi);
	double spsi = sin(psi);

	dcm << cpsi * ctheta,
		cpsi* stheta* sphi - spsi * cphi,
		cpsi* stheta* cphi + spsi * sphi,

		spsi* ctheta,
		spsi* stheta* sphi + cpsi * cphi,
		spsi* stheta* cphi - cpsi * sphi,

		-stheta,
		ctheta* sphi,
		ctheta* cphi;

	return dcm;
}

////////////////////////////
////EKF FUNCTIONS 

Vector15d get_xdot(const Vector15d& x, const Vector3d& g, const Vector3d& a_body_measured, const Vector3d& omega)
{
	//note that a_measured is the IMU measurement, as measured from the body frame. No gravity included, and still in body vectors.
	//x is expected to be attitude, pos, v, accel bias, gyro bias
	Vector15d xdot;
	Matrix3d T; //This is the transform from body rates to euler rates
	T << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));
	xdot.block(0, 0, 3, 1) = T * (omega - x.block(12,0,3,1)); //subtracting bias
	xdot.block(3, 0, 3, 1) = x.block(6, 0, 3, 1);
	xdot.block(6, 0, 3, 1) = dcmI_B(x(0), x(1), x(2)) * (a_body_measured-x.block(9,0,3,1)) + g; //subtracting bias
	xdot.block(9,0,6,1) = Eigen::Matrix<double, 6, 1>::Zero();
	return xdot;
}

Matrix15d jacobian(const Vector15d& x, const Vector3d& g, const Vector3d& a_body_measured, const Vector3d& omega)
{ 
	Matrix15d A;
	//generally, the attitude derivates are complex, rest are either 0 or 1
	A.setZero();

	double phi = x(0);
	double theta = x(1);
	double psi = x(2);

	// Cache trig
	double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double t_theta = stheta / ctheta;  
	double cpsi = cos(psi);
	double spsi = sin(psi);

	//attitude derivatives
	A.block(0, 0, 1, 3) << omega(1) * cphi * t_theta - omega(2) * sphi * t_theta,
		(omega(1) * sphi + omega(2) * cphi) / (ctheta * ctheta), 0; //phi
	A.block(1, 0, 1, 3) << -omega(1) * sphi - omega(2) * cphi, 0, 0;//theta
	A.block(2, 0, 1, 3) << omega(1) * cphi / ctheta - omega(2) * sphi / ctheta,
		omega(1)* sphi / ctheta * t_theta + omega(2) * cphi / ctheta * t_theta, 0; // psi

	A.block(0, 12, 3, 3) << -1, -sphi * t_theta, -cphi * t_theta,
		0, -cphi, sphi,
		0, -sphi / ctheta, -cphi / ctheta; //gyro bias

	//dposdot/datt = 0
	Matrix3d storage; //used to store stuff for calcs, reused over and over

	//dVdot/datt

	//dVdot/dphi
	storage << 0, cpsi* stheta* cphi + spsi * sphi, -cpsi * stheta * sphi + spsi * cphi,
		0, spsi* stheta* cphi - cpsi * sphi, -spsi * stheta * sphi - cpsi * cphi,
		0, ctheta* cphi, -ctheta * sphi;
	A.block(6, 0, 3, 1) = storage * a_body_measured;

	storage << -cpsi * stheta, cpsi* ctheta* sphi, cpsi* ctheta* cphi,
		-spsi * stheta, spsi* ctheta* sphi, spsi* ctheta* cphi,
		-ctheta, -stheta * sphi, -stheta * cphi;
	A.block(6, 1, 3, 1) = storage * a_body_measured;

	storage << -spsi * ctheta, -spsi * stheta * sphi - cpsi * cphi, -spsi * stheta * cphi + cpsi * sphi,
		cpsi* ctheta, cpsi* stheta* sphi - spsi * cphi, cpsi* stheta* cphi + spsi * sphi,
		0, 0, 0;
	A.block(6, 2, 3, 1) = storage * a_body_measured;

	//dv/dbias
	A.block(6, 9, 3, 3) << -cpsi * ctheta, -cpsi * stheta * sphi - spsi * cphi, -cpsi * stheta * sphi + spsi * sphi,
		-spsi * ctheta, -spsi * stheta * sphi + cpsi * cphi, -spsi * stheta * cphi - cpsi * sphi,
		stheta, -ctheta * sphi, -ctheta * cphi;

	//Done with attitudes. n e d partials are trivial. Only thing left is a few ones
	A(3, 6) = 1;
	A(4, 7) = 1;
	A(5, 8) = 1;


	return A;



}
Eigen::Matrix<double, 15, 12> noise_coupling(const Vector15d& x)
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

Vector3d sim_imu_accels(const Vector12d& x_true, const Vector3d& commanded_body_accel, const Vector3d& alpha, const Vector3d& r, const Vector3d& imunoise)
{
	//x_true is attitude, body rates, pos, v
	// commanded_body_accels is true non-gravity acceleration in the body frame, at the COM of the object. Essentially, for quadrotor, specific thrust. 
	// r = IMU - CG (body frame). 
	// In EKF, we use -r to map IMU measurement back to CG.	
	//returns IMU acceleration reading for a given body acceleration at CG. 
	Matrix3d to_inertial = dcmI_B(x_true(0), x_true(1), x_true(2)); 
	Matrix3d to_body = to_inertial.transpose(); 
	Vector3d omega_measured = x_true.block(3, 0, 3, 1); 
	Vector3d g; g << 0, 0, 9.81; 
	Vector3d accel_cgI = to_inertial * commanded_body_accel + g;
	double ground = 0.0; double penetration = x_true(8) - ground; 
	if (penetration > 0) 
	{ 
		double k_ground = 1000; 
		double b_ground = 50; 
		accel_cgI(2) += -k_ground * penetration - b_ground * x_true(11); 
	} 
	Vector3d body_transfer = alpha.cross(r) + omega_measured.cross(omega_measured.cross(r)); 
	Vector3d off_cg_accels = to_body * accel_cgI + body_transfer; 
	Vector3d a_measured = off_cg_accels - to_body * g + imunoise;
	//moving imu stuff to COM Vector3d off_cg_accels = to_body * off_cg_I_accel; Vector3d a_measured = off_cg_accels - to_body * g + imunoise;
	return a_measured;
}
Vector3d sim_gyro_rates(const Vector12d& x_true, const Vector3d& gyronoise)
{
	return x_true.block(3, 0, 3, 1) + gyronoise;
}

Vector4d sim_measurement(const Vector4d& x_true_measured, const Vector4d& m_noise)
{
	return x_true_measured + m_noise;
}

//GET DYNAMICS DOES NOT USE THE SAME STATES AS EKF. get_xdot is for the KALMAN FILTER. This is for propagating true 12d state: attitude, body rates, pos, vel
Vector12d get_dynamics(const Vector12d& x, const Vector3d& g, double m, const Vector3d& inertias, double thrust, const Vector3d& moments)
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
	static std::mt19937 gen(std::random_device{}());
	static std::normal_distribution<double> dist(0.0, 1.0);
	Vector12d noise;
	for (int i = 0; i < 12; i++)
	{
		noise(i) = dist(gen);
	}
	return noise;
}

Vector4d noise4d()
{
	static std::mt19937 gen(std::random_device{}());
	static std::normal_distribution<double> dist(0.0, 1.0);
	Vector4d noise;
	for (int i = 0; i < 4; i++)
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

double clamp(double x, double lo, double hi) 
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

Vector4d throttle2pwm(Eigen::Vector4d throttles)
{
	Eigen::Vector4d pwms = (throttles.array() + 1.0) * 1000;
	return pwms;
}

