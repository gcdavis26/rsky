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
#include "network.h"
#include "RCInput.h"
#include "MotorDriver.h"
#include <chrono>
#include <thread>
#include <functional>
#include "IMUHandler.h"


int main() {

	openPort();

////////////////////////////////////////////////////////////////////////////////////////// SETUP

	//BASE SETUP OF PHYSICS
	Vector3d g;
	double m = .75;
	g << 0, 0, 9.81;
	double tc = .028;
	double Kt = 4.0;
	double Kq = 2.35;
	double d_arm = .15;
	Vector3d r = Vector3d::Zero(); //can be changed to add an IMU offset

	//SETUP OF ROOM
	Vector2d n_bounds;
	n_bounds << 0, 30 * .3048;
	Vector2d e_bounds;
	e_bounds << 0, 15 * .3048;
	double cruise = .75;
	double yaw_rate = .5; //rad/s, about 28 degrees per second at max
	double takeoff_height = 2;

	///////////////////
	//GAINS CHANGE HERE
	///////////////////
	Vector3d Kp_outer;
	Kp_outer << 0.7, 0.7, 2;
	Vector3d Kd_outer;
	Kd_outer << 1.5, 1.5, 3;
	Vector3d Kp_inner;
	Kp_inner << .5, .5, .5;
	Vector3d Kd_inner;
	Kd_inner << .1,.1,.1;
	std::pair<double, double> T_sat;

	////////////////
	//SATURATION
	////////////////
	T_sat.first = .2 * m * g(2); //min
	T_sat.second = 2.0 * m * g(2); //max
	std::pair<double, double> acc_sat;
	acc_sat.first = g(2) / 4; //n e
	acc_sat.second = g(2); //d
	double max_angle = 15 * M_PI / 180;

	//MIXER
	Eigen::Matrix4d mixer;
	mixer << Kt, Kt, Kt, Kt,
		-d_arm * Kt, -d_arm * Kt, d_arm* Kt, d_arm* Kt,
		d_arm* Kt, -d_arm * Kt, -d_arm * Kt, d_arm* Kt,
		Kq, -Kq, Kq, -Kq;

	//SETUP OF STOCHASTIC STUFF FOR EKF
	Vector12d sigmaw;
	sigmaw << .1, .1, .1, .01, .01, .01, 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4; //gyro,accelerometer, bias_accel, bias_gyro
	Vector3d sigmav;
	sigmav << .001, .001, .001; //n,e,d

	//THESE SHOULDNT BE ZERO.
	Vector3d accel_bias;
	accel_bias.setZero();
	Vector3d gyro_bias;
	gyro_bias.setZero();
	//MAYBE WAIT FOR SOME PERIOD OF TIME HERE BEFORE INITITALIZING CLASSES


	//EKF SETUP, INITIAL MEASUREMENTS
	EKF ekf(r, sigmaw, sigmav);
	//CONTROLLER SETUP
	Controller controller(Kp_outer, Kd_outer, Kp_inner, Kd_inner, T_sat, acc_sat, max_angle, m);
	//controller.update(x_true); //for testing


	//MOTOR SETUP
	MotorDriver motordriver;
	RCInputHandler rc_controller;


	//These are derivatives and controls etc at initial state
	Vector3d imu_omega; //read measurements from IMU
	Vector3d imu_accels; //IMU
	Vector4d controls;
	Vector4d motor_cmds;
	Vector4d forces;

	//SOME SORT OF TELEMETRY STUFF HERE
	IMUHandler imu;
	Eigen::Matrix<double,5,1> mocapData = readDatalink();
	Eigen::Matrix<double, 6, 1> imu_data = imu.update();

	imu_accels = imu_data.head<3>();
	imu_omega = imu_data.tail<3>();

	Vector3d measurement = mocapData.head<3>(); //Optitrack
	ekf.initialize(measurement, imu_omega, imu_accels, accel_bias, gyro_bias);
	Vector12d x = ekf.getControlState();
	Guidance guidance(x, n_bounds, e_bounds, 4, cruise,yaw_rate, takeoff_height, 1, .5); //initialize guidance system

	///////////////////
	// //Frequencies
	///////////////////
	double process_freq = 200.0;
	double m_freq = 50.0;
	//control loop should be 400 hz, but we don't need to limit it 

	auto t_ref = std::chrono::system_clock::now(); //main loop clock
	auto measurement_t = std::chrono::system_clock::now(); //measurement clock
	auto process_t = std::chrono::system_clock::now(); //process clock

////////////////////////////////////////////////////////////////////////////////////////// SETUP OVER

	while (true)
	{
		//WAIT FUNCTION TO PAD THE .01 seconds.
		auto current_time = std::chrono::system_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::seconds>(current_time - process_t);
		if (dt.count() >= 1 / process_freq)
		{
			ekf.estimate(dt.count()); //predict state at current time step
			imu_data = imu.update();
			imu_omega = imu_data.block(0, 0, 3, 1);
			imu_accels = imu_data.block(3, 0, 3, 1);
			ekf.imureading(imu_omega, imu_accels, dt.count());
			process_t = current_time;
		}

		//there will be some time lag between the ekf estimate and the measurement update, but this should be small enough to ignore
		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::seconds>(current_time - measurement_t);
		if (dt.count() >= 1 / m_freq) // 5th value in mocapData matrix is valid bit
		{
			Eigen::Matrix<double, 5, 1> opti_data = readDatalink();
			if (opti_data(4) == true) //if valid
			{
				measurement = opti_data.block(0, 0, 3, 1);
				ekf.update(measurement); //update our state estimate
			}
			else
			{
				//maybe have some sort of warning, telemetry, etc.
			}
			
		}
		x = ekf.getControlState();
		controller.update(x); //update internal control state

		//get commanded quantities
		Eigen::Matrix<double, 6, 1> rc_data = rc_controller.read_ppm_vector();
		Vector4d v_cmds = guidance.manualCommands(rc_data);
		controls = controller.manualControl(v_cmds); //get forces
		motor_cmds = mixer.inverse() * controls; //get motor commands
		motor_cmds = (motor_cmds.array().min(1)).max(0); //force motor throttle between 0 and 1
		//COMMAND THE MOTORS

		if (static_cast<int>(rc_data(5)) == 2000)
		{
			motordriver.~MotorDriver();
		}
		else
		{
			motordriver.command(motor_cmds);
		}


			
	}

	return 0;
}