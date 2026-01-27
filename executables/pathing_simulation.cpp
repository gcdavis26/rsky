#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include "controller.h"
#include "ekf.h"
#include "guidance.h"
#include "math_utils.h"
#include <cmath>
#include <random>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <chrono>
#include <thread>

int main() {

	//BASE SETUP OF ROOM
	Eigen::Vector3d pos0 = Eigen::Vector3d::Zero();
	double psi0 = 0;

	Eigen::Vector2d n_bounds;
	n_bounds << 0, 30 * .3048;
	Eigen::Vector2d e_bounds;
	e_bounds << 0, 15 * .3048;
	double cruise = 1;
	double takeoff_height = 2;
	Eigen::Vector3d target_pos;
	target_pos << 3, 6, 0;



	//BASE SETUP OF PHYSICS
	Eigen::Matrix<double, 12, 1> x_true = Eigen::Matrix<double, 12, 1>::Zero();
	x_true.block(6, 0, 3, 1) = pos0;
	Eigen::Vector3d g;
	double m = .75;
	Eigen::Vector3d inertias;
	inertias << .00756, .00757, .01393;
	g << 0, 0, 9.81;
	double freq = 100.0;
	double m_freq = 100.0;
	double deltat = 1.0 / freq;
	double tc = .028;
	double Kt = 4.0;
	double Kq = 2.35;
	double d_arm = .15;
	Eigen::Vector3d r = Eigen::Vector3d::Zero(); //can be changed to add an IMU offset


	///////////////////
	//GAINS CHANGE HERE
	///////////////////
	Eigen::Vector3d Kp_outer;
	Kp_outer << 0.7, 0.7, 2;
	Eigen::Vector3d Kd_outer;
	Kd_outer << 1.5, 1.5, 3;
	Eigen::Vector3d Kp_inner;
	Kp_inner << .5, .5, .5;
	Eigen::Vector3d Kd_inner;
	Kd_inner << .1,.1,.1;
	std::pair<double, double> T_sat;

	/////////////////
	//SATURATION
	////////////////
	T_sat.first = .2 * m * g(2); //min
	T_sat.second = 2.0 * m * g(2); //max
	std::pair<double, double> acc_sat;
	acc_sat.first = g(2) / 4; //xy
	acc_sat.second = g(2); //z
	double max_angle = 15 * M_PI / 180;

	//MIXER
	Eigen::Matrix4d mixer;
	mixer << Kt, Kt, Kt, Kt,
		-d_arm * Kt, -d_arm * Kt, d_arm* Kt, d_arm* Kt,
		d_arm* Kt, -d_arm * Kt, -d_arm * Kt, d_arm* Kt,
		Kq, -Kq, Kq, -Kq;

	//SETUP OF STOCHASTIC STUFF FOR EKF
	Eigen::Matrix<double, 12, 1> sigmaw;
	sigmaw << .1, .1, .1, .01, .01, .01, 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4; //gyro,accelerometer, bias_accel, bias_gyro
	Eigen::Vector3d sigmav;
	sigmav << .001, .001, .001; //n,e,d
	Eigen::Matrix<double, 12, 1> w;
	w = noise12d().asDiagonal() * sigmaw; 
	Eigen::Vector3d v;
	v = noise3d().asDiagonal() * sigmav;

	//EKF SETUP, INITIAL MEASUREMENTS
	Eigen::Matrix<double, 15, 1> x0 = Eigen::Matrix<double, 15, 1>::Zero();
	x0.block(3, 0, 3, 1) = pos0;
	x0(2) = psi0;
	Eigen::Vector3d truth_measured = x0.block(2, 0, 3, 1);
	Eigen::Vector3d measurement = sim_measurement(truth_measured, v);
	x0.block(2, 0, 3, 1) = measurement; 
	Eigen::Vector3d imu_accels = sim_imu_accels(x_true, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), w.block(3, 0, 3, 1));
	Eigen::Vector3d imu_omega = sim_gyro_rates(x_true, w.block(0, 0, 3, 1));
	EKF ekf(r, x0, imu_omega, imu_accels, sigmaw, sigmav, freq);
	Eigen::Matrix<double, 12, 1> x = ekf.getControlState();

	//CONTROLLER SETUP
	Controller controller(Kp_outer, Kd_outer, Kp_inner, Kd_inner, T_sat, acc_sat, max_angle, m);
	controller.update(x);
	//controller.update(x_true); //for testing

	Guidance guidance(x,n_bounds, e_bounds, 5, cruise, takeoff_height,.1);
	double t = 0;
	


	//These are derivatives and controls etc at initial state
	Eigen::Matrix<double, 10, 1> commands = guidance.getTarget(x);
	Eigen::Vector4d controls = controller.achieveState(commands(0), commands.block(1, 0, 3, 1), commands.block(4, 0, 3, 1), commands.block(7, 0, 3, 1));
	Eigen::Vector4d motor_cmds = mixer.inverse() * controls;
	motor_cmds = (motor_cmds.array().min(1)).max(0);
	Eigen::Vector4d forces = mixer * motor_cmds;
	Eigen::Vector3d specific_thrust;
	specific_thrust << 0,0, -forces(0);
	specific_thrust = specific_thrust / m;
	Eigen::Matrix<double, 12, 1> xdot = get_dynamics(x_true, g, m, inertias, forces(0), forces.block(1, 0, 3, 1));

	std::cout << "Simulation has begun" << std::endl;
	std::cout << "Initial controller state estimate: " << x << std::endl;
	std::cout << "Initial true state: " << x_true << std::endl;
	std::cout << "Initial measurement: " << measurement << std::endl;
	std::cout << "Initial IMU (omega, accels):" << imu_omega << std::endl << imu_accels << std::endl;
	std::cout << "Initial motor commands: " << motor_cmds << std::endl;


	//recording data
	std::ofstream outfile("sim_results.csv");
	
	outfile << "t,phi,phi_est,theta,theta_est,psi,psi_est,p,p_est,q,q_est,r,r_est,n,n_est,e,e_est,d,d_est,vn,vn_est,ve,ve_est,vd,vd_est" << std::endl;
	outfile << t << ',';
	for (int l = 0; l < 12; l++)
	{
		outfile << x_true(l) << ',' << x(l);

		if (l != 11)
		{
			outfile << ',';
		}
		else
		{
			outfile << std::endl;
		}
	}

	for (int k =1; k < 80*freq+1; k++)
	{
		t += deltat; //propagate truth
		x_true += xdot * deltat;

		//take new measurements
		w = noise12d().asDiagonal() * sigmaw;
		imu_omega = sim_gyro_rates(x_true, w.block(0, 0, 3, 1));
		imu_accels = sim_imu_accels(x_true, specific_thrust, xdot.block(3,0,3,1), r, w.block(3, 0, 3, 1));
		//estimate
		ekf.estimate(imu_omega, imu_accels); //note that the estimate stores the new imu readings, but uses the older ones in the prediction
		
		v = noise3d().asDiagonal() * sigmav;
		truth_measured << x_true(6), x_true(7), x_true(8);
		measurement = sim_measurement(truth_measured, v);
		ekf.update(measurement);
		
		x = ekf.getControlState();
		controller.update(x); //update internal control state
		//controller.update(x_true); //for testing

		//guidance and control
		commands = guidance.getTarget(x); //get commanded quantities
		controls = controller.achieveState(commands(0), commands.block(1, 0, 3, 1), commands.block(4, 0, 3, 1), commands.block(7, 0, 3, 1)); //get forces
		motor_cmds = mixer.inverse() * controls; //get motor commands
		motor_cmds = (motor_cmds.array().min(1)).max(0); //force motor throttle between 0 and 1
		forces = mixer * motor_cmds; //get actual forces
		specific_thrust << 0, 0, -forces(0);
		specific_thrust = specific_thrust / m;
		xdot = get_dynamics(x_true, g, m, inertias, forces(0), forces.block(1, 0, 3, 1)); //true state derivative
		
		outfile << t << ',';
		for (int l = 0; l < 12; l++)
		{
			outfile << x_true(l) << ',' << x(l);

			if (l != 11)
			{
				outfile << ',';
			}
			else
			{
				outfile << std::endl;
			}
		}
		//std::cout << "Specific thrust: " << specific_thrust << std::endl << "IMU: " << imu_accels << std::endl << "NEXT: " << std::endl;
		//std::cout << "Measurement: " << measurement << std::endl << "Actual: " << truth_measured << std::endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	outfile.close();
	return 0;
}