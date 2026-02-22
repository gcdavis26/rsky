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

#include "IMUHandler.h"
#include "telemetrypacket.h"
#include "UDPSender.h"
#include "network.h"
#include "MotorDriver.h"
#include "RCInput.h"
#include "telemetrypacket.h"
#include "UDPSender.h"

#include <chrono>
#include <thread>


int main() {

	openPort();
	//BASE SETUP OF ROOM
	Vector3d pos0 = Vector3d::Zero();
	double psi0 = 0;

	Vector2d n_bounds;
	n_bounds << 0, 30 * .3048;
	Vector2d e_bounds;
	e_bounds << 0, 15 * .3048;
	double cruise = .75;
	double yaw_rate = .5; //rad/s, about 28 degrees per second at max
	double takeoff_height = 2;
	Vector3d target_pos;
	target_pos << 3, 6, 0;

	//BASE SETUP OF PHYSICS
	Vector3d g;
	double m = .75;
	Vector3d inertias;
	inertias << .00756, .00757, .01393;
	g << 0, 0, 9.81;
	double tc = .028;
	double Kt = 4.0;
	double Kq = 2.35;
	double d_arm = .15;
	Vector3d r = Vector3d::Zero(); //can be changed to add an IMU offset


	///////////////////
	//GAINS CHANGE HERE
	///////////////////
	Vector3d Kp_outer;
	Kp_outer << 0.5, 0.5, 2;
	Vector3d Kd_outer;
	Kd_outer << 1, 1, 3;
	Vector3d Kp_inner;
	Kp_inner << .5, .5, .75;
	Vector3d Kd_inner;
	Kd_inner << .1, .1, .1;
	std::pair<double, double> T_sat;

	/////////////////
	//SATURATION
	////////////////
	T_sat.first = .2 * m * g(2); //min
	T_sat.second = 2.0 * m * g(2); //max
	std::pair<double, double> acc_sat;
	acc_sat.first = g(2) / 4; //n e
	acc_sat.second = g(2); //d
	double max_angle = 10 * M_PI / 180;

	//MIXER
	Matrix4d mixer;
	mixer << Kt, Kt, Kt, Kt,
		-d_arm * Kt, -d_arm * Kt, d_arm* Kt, d_arm* Kt,
		d_arm* Kt, -d_arm * Kt, -d_arm * Kt, d_arm* Kt,
		Kq, -Kq, Kq, -Kq;
	Matrix4d mixer_inverse = mixer.inverse();
	//1 2
	//4 3 motor config
	//1 and 3 ccw, 2 and 4 are cw

	//SETUP OF STOCHASTIC STUFF FOR EKF
	Vector12d sigmaw;
	sigmaw << .1, .1, .1, .01, .01, .01, 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4; //gyro,accelerometer, bias_accel, bias_gyro
	Vector3d sigmav;
	sigmav << .001, .001, .001; //n,e,d

	IMUHandler imu;

	Eigen::Matrix<double, 12, 1> biases = imu.initialize();

	Vector3d accel_bias;
	Vector3d gyro_bias;
	Vector3d gyro_std_dev;
	Vector3d accel_std_dev;
	
	accel_bias = biases.segment<3>(3);
	//accel_bias << -0.21, 0.5, 9.81-9.7;
	//accel_bias.setZero();
	
	gyro_bias = biases.head<3>();
	//gyro_bias << -0.29,-0.26 -0.33;
	//gyro_bias.setZero();

	gyro_std_dev = biases.segment<3>(6);
	accel_std_dev = biases.tail<3>();

	//EKF creation, initialization
	EKF ekf(r, sigmaw, sigmav); //ekf created

	Eigen::Matrix<double, 6, 1> imu_data = imu.update();
	Eigen::Matrix<double, 8, 1> opti_data = readDatalink();
	Vector3d measurement = opti_data.block(0, 0, 3, 1);
	Vector3d imu_accels = imu_data.block(3, 0, 3, 1);
	Vector3d imu_omega = imu_data.block(0, 0, 3, 1);
	ekf.initialize(measurement, imu_omega, imu_accels, accel_bias, gyro_bias); //initializing with values. Can be done after a wait as well.
	Vector12d x = ekf.getControlState();

	//CONTROLLER SETUP
	Controller controller(Kp_outer, Kd_outer, Kp_inner, Kd_inner, T_sat, acc_sat, max_angle, m);
	controller.update(x);
	Guidance guidance(x, n_bounds, e_bounds, 4, cruise, yaw_rate, takeoff_height, 1, .5); //initialize guidance system
	//controller.update(x_true); //for testing
	//These are derivatives and controls etc at initial state
	Vector4d controls;
	Vector4d motor_cmds;
	MotorDriver motordriver;
	RCInputHandler rc_controller;

	///////////////////
	// //Frequencies
	///////////////////
	double process_freq = 200.0;
	double m_freq = 50.0;
	//control loop should be 400 hz, but we don't need to limit it 
	double t = 0;
	auto t_ref = std::chrono::system_clock::now(); //total time reference
	auto process_t = std::chrono::system_clock::now(); //process clock
	auto measurement_t = std::chrono::system_clock::now(); //measurement clock
	auto telemetry_t = std::chrono::system_clock::now(); //telemetry clock
	int cycles = 0;
	double sim_time = 120;
	UDPSender sender("127.0.0.1", 5000);
	TelemetryPacket pkt;
	uint32_t packetCounter = 0;

	while (true)
	{
		//process model
		auto current_time = std::chrono::system_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - process_t);
		double dt_secs = dt.count() / 1e6;

		if (dt_secs >= 1 / process_freq)
		{

			ekf.estimate(dt.count()); //predict state at current time step
			imu_data = imu.update();
			imu_omega = imu_data.block(0, 0, 3, 1);
			imu_accels = imu_data.block(3, 0, 3, 1);
			ekf.imureading(imu_omega, imu_accels, dt.count());
			process_t = current_time;
		}

		//measurement model
		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - measurement_t);
		dt_secs = dt.count() / 1e6;

		if (dt_secs >= 1 / m_freq)
		{
			Eigen::Matrix<double, 8, 1> opti_data = readDatalink();
			if (opti_data(7) == 1.0) //if valid
			{
				measurement = opti_data.block(0, 0, 3, 1);
				ekf.update(measurement); //update our state estimate
			}
			else
			{
				pkt.flags = 0;
			}
			measurement_t = current_time;
		}


		//controls and motors
		x = ekf.getControlState();
		controller.update(x); //update internal control state
		//controller.update(x_true); //for testing

		//guidance and control
		Eigen::Matrix<double, 6, 1> rc_data = rc_controller.read_ppm_vector();

		Vector4d v_cmds = guidance.manualCommands(rc_data);
		controls = controller.manualControl(v_cmds); //get forces
		motor_cmds = mixer.inverse() * controls; //get motor commands
		motor_cmds = (motor_cmds.array().min(1)).max(0); //force motor throttle between 0 and 1
		cycles += 1;

		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - telemetry_t);
		dt_secs = dt.count() / 1e6;
		if (dt_secs >= 0.1)
		{
			pkt.time = std::chrono::duration<double>(
				std::chrono::steady_clock::now().time_since_epoch()
			).count();

			std::memcpy(pkt.state, ekf.getState().data(), STATE_SIZE * sizeof(double));
			pkt.counter = packetCounter++;

			sender.send(pkt);
			pkt.flags = 1; //reset flag
			telemetry_t = current_time;
		}
		if (static_cast<int>(rc_data(5)) == 2000)
		{
			motordriver.~MotorDriver();
		}
		else
		{
			motordriver.command(motor_cmds);
		}

	}
	auto total_t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t_ref);
	std::cout << (cycles / (total_t.count() / 1e6));

	return 0;
}
