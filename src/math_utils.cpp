#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include "math_utils.h"

using namespace std;
Eigen::Matrix3d dcmI_B(double phi, double theta, double psi)
{
	//This returns a DCM that goes from body frame to inertial
    Eigen::Matrix<double, 3, 3> Z_rot;
	Eigen::Matrix<double, 3, 3> Y_rot;
	Eigen::Matrix<double, 3, 3> X_rot;

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

Eigen::Matrix<double, 15, 1> get_xdot(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega)
{
	//note that a_measured is the inertial acceleration, as measured from the body frame. No gravity included, and still in body vectors.
	//a_inertial_measured includes the correction due to off-cg IMU location
	Eigen::Matrix<double, 15, 1> xdot;
	Eigen::Matrix3d T; //This is the transform from body rates to euler rates
	T << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));
	xdot.block(0, 0, 3, 1) = T * omega;
	xdot.block(3, 0, 3, 1) = x.block(6, 0, 3, 1);
	xdot.block(6, 0, 3, 1) = dcmI_B(x(0), x(1), x(2)) * a_inertial_measured + g;
	xdot.block(9,0,6,1) = Eigen::Matrix<double, 6, 1>::Zero();
	return xdot;
}
//GET DYNAMICS DOES NOT USE THE SAME STATES AS X
Eigen::Matrix<double, 12, 1> get_dynamics(Eigen::Matrix<double, 12, 1> x, Eigen::Vector3d g, double m, Eigen::Vector3d inertias,Eigen::Vector3d forces, Eigen::Vector3d moments)
{
//x = n e d, vn ve vd, phi theta psi, p q r
//xdot = vn ve vd, an ae ad, euler_rates, omegadot
	double L = moments(0);
	double M = moments(1);
	double N = moments(2);
	double Ix = inertias(0);
	double Iy = inertias(1);
	double Iz = inertias(2);
	Eigen::Matrix3d to_euler; //This is the transform from body rates to euler rates
	to_euler << 1, sin(x(6))* tan(x(7)), cos(x(6))* tan(x(7)),
		0, cos(x(6)), -sin(x(6)),
		0, sin(x(6)) / cos(x(7)), cos(x(6)) / cos(x(7));
	Eigen::Matrix<double, 12, 1> xdot;
	xdot.block(0, 0, 3, 1) = x.block(3, 0, 3, 1);
	xdot.block(3, 0, 3, 1) = dcmI_B(x(6), x(7), x(8)) * forces / m + g;
	xdot.block(6, 0, 3, 1) = to_euler * x.block(9, 0, 3, 1);
	xdot(9) = ((Iy - Iz) * x(10) * x(11) + L) / Ix;
	xdot(10) = ((Iz - Ix) * x(9) * x(11) + M) / Iy;
	xdot(11) = ((Ix - Iy) * x(9) * x(10) + N) / Iz;

	return xdot;
}

Eigen::Matrix<double, 15, 15> jacobian(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d g, Eigen::Vector3d a_inertial_measured, Eigen::Vector3d omega)
{ 
	//note that a_inertial_measured is the inertial acceleration, as measured from the body frame. No gravity included, and still in body vectors.
	double eps = 1e-6;
	Eigen::Matrix<double, 15, 15> A;
	for (int i = 0; i < 15; i++)
	{
		Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15,1>::Zero();
		dx(i) = eps;
		A.col(i) = (get_xdot(x + dx, g, a_inertial_measured, omega) - get_xdot(x - dx, g, a_inertial_measured, omega)) / (2 * eps);
	}
	return A;
}
Eigen::Matrix<double, 15, 12> noise_coupling(Eigen::Matrix<double, 15, 1> x)
{
	Eigen::Matrix3d T;
	T << 1, sin(x(0))* tan(x(1)), cos(x(0))* tan(x(1)),
		0, cos(x(0)), -sin(x(0)),
		0, sin(x(0)) / cos(x(1)), cos(x(0)) / cos(x(1));

	Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();
	G.block(0, 0, 3, 3) = T;
	G.block(6, 3, 3, 3) = dcmI_B(x(0), x(1), x(2));
	G.block(9, 6, 3, 3) = Eigen::Matrix3d::Identity();
	G.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();
	
	return G;
}

double wrapPi(double angle)
{
	return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

double saturate(double command, double saturation, bool ismax = true)
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