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

#include "telemetrypacket.h"
#include "UDPSender.h"

#include <chrono>
#include <thread>

int main() {

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
	Vector12d x_true = Vector12d::Zero();
	x_true.block(6, 0, 3, 1) = pos0;
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
	Kd_inner << .1,.1,.1;
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
	Vector12d w;
	w = noise12d().cwiseProduct(sigmaw); 
	Vector3d v;
	v = noise3d().cwiseProduct(sigmav);
	Vector3d accel_bias;
	accel_bias.setZero();
	Vector3d gyro_bias;
	gyro_bias.setZero();

	//EKF creation, initialization
	EKF ekf(r, sigmaw, sigmav); //ekf created

	Vector3d truth_measured = x_true.block(3, 0, 3, 1);
	Vector3d measurement = sim_measurement(truth_measured, v);
	Vector3d imu_accels = sim_imu_accels(x_true, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), w.block(3, 0, 3, 1));
	Vector3d imu_omega = sim_gyro_rates(x_true, w.block(0, 0, 3, 1));
	ekf.initialize(measurement, imu_omega, imu_accels,accel_bias,gyro_bias); //initializing with values. Can be done after a wait as well.
	Vector12d x = ekf.getControlState();

	//CONTROLLER SETUP
	Controller controller(Kp_outer, Kd_outer, Kp_inner, Kd_inner, T_sat, acc_sat, max_angle, m);
	controller.update(x);
	//controller.update(x_true); //for testing

	Guidance guidance(x,n_bounds, e_bounds, 4, cruise, yaw_rate, takeoff_height,1,.5);

	//These are derivatives and controls etc at initial state
	Vector10d commands = guidance.getTarget(x);
	Vector4d controls = controller.achieveState(commands(0), commands.block(1, 0, 3, 1), commands.block(4, 0, 3, 1), commands.block(7, 0, 3, 1));
	Vector4d motor_cmds = mixer_inverse * controls;
	motor_cmds = (motor_cmds.array().min(1)).max(0);
	Vector4d forces = mixer * motor_cmds;
	Vector3d specific_thrust;
	specific_thrust << 0,0, -forces(0);
	specific_thrust = specific_thrust / m;
	Vector12d xdot = get_dynamics(x_true, g, m, inertias, forces(0), forces.block(1, 0, 3, 1));

	///////////////////
	// //Frequencies
	///////////////////
	double process_freq = 200.0;
	double m_freq = 50.0;
	//control loop should be 400 hz, but we don't need to limit it 
	double t = 0;
	auto t_ref = std::chrono::system_clock::now(); //total time reference
	auto t_loop = std::chrono::system_clock::now(); //main loop clock
	auto process_t = std::chrono::system_clock::now(); //process clock
	auto measurement_t = std::chrono::system_clock::now(); //measurement clock
	auto telemetry_t = std::chrono::system_clock::now(); //telemetry clock
	int cycles = 0;
	double sim_time = 30;
	UDPSender sender("127.0.0.1", 5000);
	TelemetryPacket pkt;
	uint32_t packetCounter = 0;

	while(t < sim_time)
	{
		auto current_time = std::chrono::system_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - t_loop);
		double dt_secs = dt.count() / 1e6;
		t += dt_secs; //propagate truth
		x_true += xdot * dt_secs;
		t_loop = current_time;

		//process model
		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - process_t);
		dt_secs = dt.count() / 1e6;

		if (dt_secs >= 1 / process_freq)
		{
			
			ekf.estimate(dt_secs); //predict state at current time step
			w = noise12d().cwiseProduct(sigmaw);
			imu_omega = sim_gyro_rates(x_true, w.block(0, 0, 3, 1));
			imu_accels = sim_imu_accels(x_true, specific_thrust, xdot.block(3, 0, 3, 1), r, w.block(3, 0, 3, 1));
			ekf.imureading(imu_omega, imu_accels, dt_secs);
			process_t = current_time;
		}


		//measurement model
		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - measurement_t);
		dt_secs = dt.count() / 1e6;
		
		if (dt_secs >= 1 / m_freq) 
		{
			v = noise3d().cwiseProduct(sigmav);
			truth_measured << x_true(6), x_true(7), x_true(8);
			measurement = sim_measurement(truth_measured, v);
			ekf.update(measurement);
			measurement_t = current_time;
		}


		//controls and motors
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
		cycles += 1;

		current_time = std::chrono::system_clock::now();
		dt = std::chrono::duration_cast<std::chrono::microseconds>(current_time - telemetry_t);
		dt_secs = dt.count() / 1e6;
		if (dt_secs >= 0.1)
		{
			pkt.time = std::chrono::duration<double>(
				std::chrono::steady_clock::now().time_since_epoch()
			).count();

			std::memcpy(pkt.state, x.data(), STATE_SIZE * sizeof(double));
			pkt.flags = 1;       // e.g., filter healthy
			pkt.counter = packetCounter++;

			sender.send(pkt);
			telemetry_t = current_time;
		}

	}
	auto total_t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t_ref);
	std::cout << (cycles / (total_t.count() / 1e6));

	return 0;
}