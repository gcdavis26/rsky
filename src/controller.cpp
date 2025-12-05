#include <Eigen/Dense>
#include "controller.h"
#include "math_utils.h"
#include <cmath>
#include <algorithm>
Controller::Controller(Eigen::Vector3d outer_Kp, Eigen::Vector3d outer_Kd, Eigen::Vector3d inner_Kp, Eigen::Vector3d inner_Kd, std::pair<double, double> thrust_sat,
	std::pair<double, double> acc_sat, double angle_maximum, double mass)
{
	pos_Kp = outer_Kp;
	pos_Kd = outer_Kd;
	att_Kp = inner_Kp;
	att_Kd = inner_Kd;
	T_sat = thrust_sat;//min,max
	a_sat = acc_sat; //max xy, max z
	max_angle = angle_maximum;
	control_state = Eigen::Matrix<double, 12, 1>::Zero();
	g << 0, 0, 9.81;
	m = mass;
}

void Controller::update(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d omega)
{
	//in simulation, must call navigation update before controller update, otherwise controller will be using old info.
	control_state.block(0, 0, 3, 1) = x.block(3, 0, 3, 1);
	control_state.block(3, 0, 3, 1) = x.block(6, 0, 3, 1);
	control_state.block(6, 0, 3, 1) = x.block(0, 0, 3, 1);
	control_state.block(9, 0, 3, 1) = omega;
}

void Controller::update(Eigen::Matrix<double, 12, 1> x) //This is from no ekf simulation
{
	control_state = x;
}


Eigen::Vector3d Controller::outer_achievePos(Eigen::Vector3d p_des, Eigen::Vector3d v_des)
{
	//outer loop
	Eigen::Vector3d e_pos = p_des - control_state.block(0, 0, 3, 1);
	Eigen::Vector3d e_vel = v_des - control_state.block(3, 0, 3, 1);
	Eigen::Vector3d a_des = e_pos.array() * pos_Kp.array() + e_vel.array() * pos_Kd.array();
	a_des(0) = saturate(a_des(0), a_sat.first);
	a_des(1) = saturate(a_des(1), a_sat.first);
	a_des(2) = saturate(a_des(2), a_sat.second);

	Eigen::Vector3d a_des_body = dcmI_B(control_state(6), control_state(7), control_state(8)).transpose() * a_des;
	double phi_des = saturate(a_des_body(1) / g(2),max_angle);
	double theta_des = saturate(-a_des_body(0) / g(2),max_angle);
	// az = -T*cos(phi)*cos(theta)/m + g
	double T_des = (g(2) - a_des_body(2)) * m / (cos(phi_des) * cos(phi_des));
	T_des = saturate(T_des, T_sat.first, false);//making sure greater than min thrust
	T_des = saturate(T_des, T_sat.second); //less than max thrust
	Eigen::Vector3d output;
	output << phi_des, theta_des, T_des;
	return output;
}

Eigen::Vector3d Controller::inner_achieveAtt(Eigen::Vector3d att_des, Eigen::Vector3d omega_des)
{
	Eigen::Vector3d e_att = att_des - control_state.block(6, 0, 3, 1);
	Eigen::Vector3d e_omega = omega_des - control_state.block(9, 0, 3, 1);
	Eigen::Vector3d moments = e_att.array() * att_Kp.array() + e_omega.array() * att_Kd.array();
	return moments;
}

Eigen::Matrix<double, 4, 1> Controller::achieveState(Eigen::Vector3d p_des, Eigen::Vector3d v_des, double psi_des, Eigen::Vector3d omega_des) //returns T L M N
{
	Eigen::Vector3d outer_output = outer_achievePos(p_des, v_des);
	Eigen::Vector3d att_des;
	att_des << outer_output(0), outer_output(1), psi_des;
	Eigen::Vector3d moments = inner_achieveAtt(att_des, omega_des);
	Eigen::Matrix<double, 4, 1> commands;
	commands(0) = outer_output(2);
	commands.block(1, 0, 3, 1) = moments;
	return commands;
}

Eigen::Matrix<double, 12, 1> Controller::getState()
{
	return control_state;
}