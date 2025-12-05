#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include "ekf.h"
#include "math_utils.h"
#include <cmath>
#include <random>
//telemetry stuff
#include <iostream>
#include <fstream>
#include <string>

/*
This test is accelerating forward for 2 seconds at 1 m/s^2 while climbing, traveling for 3 more seconds,
then make a turn. 
*/

Eigen::Matrix<double, 6, 1> path(double t) //gives out true body rates and body accels for the path
{
	Eigen::Matrix<double, 6, 1> commands = Eigen::Matrix<double, 6, 1>::Zero(); //body rates, body accels
	if (t <= 2.0)
	{
		commands << 0, 0, 0, .5, 0, -.3;
	}
	else if (t <= 5.0)
	{
		commands << 0, 0, 0, 0, 0, 0;
	}
	else if (t <= 25)
	{
		commands << 0, 0, M_PI/20, 0, .25, 0;
	}
	else if (t <= 27)
	{
		commands << 0, 0, 0, 0, 0, .3;
	}
	return commands; //body rates, body accels
}

Eigen::Vector3d simulate_accelerometer(Eigen::Matrix<double, 15, 1> x_true, Eigen::Vector3d commanded_body_accel, Eigen::Vector3d imunoise)
{
	Eigen::Vector3d g;
	g << 0, 0, 9.81;
	Eigen::Vector3d a_measured = commanded_body_accel - dcmI_B(x_true(0), x_true(1), x_true(2)).transpose() * g + imunoise;
	return a_measured;
}

int main() {
	Eigen::Matrix<double, 15, 1> x_true = Eigen::Matrix<double, 15, 1>::Zero();
	double freq = 100;
	double deltat = 1 / freq;
	Eigen::Vector3d g = Eigen::Vector3d::Zero();
	g(2) = 9.81;
	Eigen::Vector3d r = Eigen::Vector3d::Zero(); //can be changed to add an IMU offset
	Eigen::Matrix<double, 12, 1> sigmaw;
	sigmaw << .1, .1, .1, .01, .01, .01, 1e-2,1e-2,1e-2,1e-4,1e-4,1e-4; //gyro,accelerometer, bias_accel, bias_gyro
	Eigen::Matrix<double, 4, 1> sigmav;
	sigmav << M_PI / 180, .001, .001, .001; //psi, n,e,d
	
	//for noise simulation
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dist(0.0, 1.0);
	
	//making w
	Eigen::Matrix<double, 12, 1> noise12d;
	for (int i = 0; i < 12; i++)
	{
		noise12d(i) = dist(gen);
	}
	Eigen::Matrix<double, 12, 1> w = noise12d.asDiagonal() * sigmaw / sqrt(deltat);
	//making v
	Eigen::Matrix<double, 4, 1> noise4d;
	noise4d << dist(gen), dist(gen), dist(gen), dist(gen);
	Eigen::Matrix<double, 6, 1> commands = path(0);
	Eigen::Vector3d imu_accels = simulate_accelerometer(x_true, commands.block(3, 0, 3, 1), w.block(3, 0, 3, 1));
	Eigen::Vector3d imu_omega = commands.block(0, 0, 3, 1) + w.block(0, 0, 3, 1);
	Eigen::Vector3d true_measured_accels = simulate_accelerometer(x_true, commands.block(3, 0, 3, 1), Eigen::Vector3d::Zero());

	Eigen::Matrix<double, 15, 1> x0 = Eigen::Matrix<double,15,1>::Zero();
	x0.block(2, 0, 4, 1) = noise4d.asDiagonal() * sigmav;
	EKF ekf(r, x0, imu_accels,imu_omega,sigmaw, sigmav, freq);
	
	std::ofstream truth_file("sim_truths.csv");
	std::ofstream ekf_file("sim_estimates.csv");
	ekf_file << "t,phi,theta,psi,n,e,d,vn,ve,vd,b_ax,b_ay,b_az,b_p,b_q,b_r" << std::endl;
	truth_file << "t,phi,theta,psi,n,e,d,vn,ve,vd,b_ax,b_ay,b_az,b_p,b_q,b_r" << std::endl;

	for (int i = 1; i <= 27 * freq; i++)
	{
		double t = i * deltat;
		for (int j = 0; j < 12; j++)
		{
			noise12d(j) = dist(gen);
		}
		//simulating IMU measurements
		w = noise12d.asDiagonal() * sigmaw / sqrt(deltat); 
		imu_accels = simulate_accelerometer(x_true, commands.block(3,0,3,1), w.block(3, 0, 3, 1));
		imu_omega = commands.block(0, 0, 3, 1) + w.block(0, 0, 3, 1);
		ekf.estimate(imu_omega, imu_accels);
		
		//truth integration
		//might want to have some of the commands change at the midpoint eval. I don't think it really matters.
		true_measured_accels = simulate_accelerometer(x_true, commands.block(3, 0, 3, 1), Eigen::Vector3d::Zero());
		//we're using true measured because xdot expects an IMU input for accels: Thus it needs to lack gravity.
		Eigen::Matrix<double,15,1> xdot_true1 = get_xdot(x_true, g, true_measured_accels, commands.block(0, 0, 3, 1));
		Eigen::Matrix<double, 15, 1> xdot_true2 = get_xdot(x_true + xdot_true1 * deltat / 2, g, true_measured_accels, commands.block(0, 0, 3, 1));
		x_true += deltat * xdot_true2;
		commands = path(t);

		//measurement update
		if (i % 10 == 0)
		{
			noise4d << dist(gen), dist(gen), dist(gen), dist(gen);
			Eigen::Matrix<double,4,1> m = x_true.block(2, 0, 4, 1) + noise4d.asDiagonal() * sigmav;
			ekf.update(m);
		}
		
		//saving out
		Eigen::Matrix<double,15,1> x_hat = ekf.getState();
		ekf_file << t<<',';
		truth_file << t<<',';
		for (int k = 0; k < 15; k++)
		{
			ekf_file << x_hat(k);
			truth_file << x_true(k);
			if (k != 14)
			{
				ekf_file << ',';
				truth_file << ',';
			}
			else
			{
				ekf_file << std::endl;
				truth_file << std::endl;
			}
		}
	}
	truth_file.close();
	ekf_file.close();
	//dist(gen)*sigmaw/sqrt(deltat);
	return 0;
}