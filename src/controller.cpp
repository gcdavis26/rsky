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
	control_state = Eigen::Matrix<double, 12, 1>::Zero(); //att, body_rate, pos, vel
	g << 0, 0, 9.81;
	m = mass;
}

void Controller::update(Eigen::Matrix<double, 12, 1> x) 
{
	control_state = x;
}


Eigen::Vector3d Controller::outer_achievePos(Eigen::Vector3d p_des, Eigen::Vector3d v_des)
{
	//outer loop
	Eigen::Vector3d e_pos = p_des - control_state.block(6, 0, 3, 1);
	Eigen::Vector3d e_vel = v_des - control_state.block(9, 0, 3, 1);
	Eigen::Vector3d a_cmd = e_pos.array() * pos_Kp.array() + e_vel.array() * pos_Kd.array();
	a_cmd(0) = saturate(a_cmd(0), a_sat.first);
	a_cmd(1) = saturate(a_cmd(1), a_sat.first);
	a_cmd(2) = saturate(a_cmd(2), a_sat.second);
	double T_cmd = (g(2) - a_cmd(2)) * m;
	T_cmd = saturate(T_cmd, T_sat.first, false);//making sure greater than min thrust
	T_cmd = saturate(T_cmd, T_sat.second); //less than max thrust
	Eigen::Matrix2d A;
	A << sin(control_state(2)),cos(control_state(2)),-cos(control_state(2)),sin(control_state(2));
	Eigen::Vector2d res =-m / T_cmd * A.transpose()*a_cmd.block(0,0,2,1); //A inverse = A transpose for this specific matrix

	double phi_cmd = saturate(res(0), max_angle);
	double theta_cmd = saturate(res(1) / g(2),max_angle);
	
	Eigen::Vector3d output;
	output << phi_cmd, theta_cmd, T_cmd;
	return output;
}

Eigen::Vector3d Controller::inner_achieveAtt(Eigen::Vector3d att_cmd, Eigen::Vector3d omega_cmd)
{
	Eigen::Vector3d e_att = att_cmd - control_state.block(0, 0, 3, 1);
	Eigen::Vector3d e_omega = omega_cmd - control_state.block(3, 0, 3, 1);
	Eigen::Vector3d moments = e_att.array() * att_Kp.array() + e_omega.array() * att_Kd.array();
	return moments;
}

Eigen::Matrix<double, 4, 1> Controller::achieveState(double psi_cmd, Eigen::Vector3d omega_cmd, Eigen::Vector3d p_cmd, Eigen::Vector3d v_cmd) //returns T L M N
{
	Eigen::Vector3d outer_output = outer_achievePos(p_cmd, v_cmd);
	Eigen::Vector3d att_cmd;
	att_cmd << outer_output(0), outer_output(1), psi_cmd;
	Eigen::Vector3d moments = inner_achieveAtt(att_cmd, omega_cmd);
	Eigen::Matrix<double, 4, 1> commands;
	commands << outer_output(2), moments;
	return commands;
}

Eigen::Matrix<double, 4, 1> Controller::innerTest(Eigen::Vector3d att_cmd, Eigen::Vector3d omega_cmd) // for inner loop control only: For test stand and manual
{
	Eigen::Vector3d moments = inner_achieveAtt(att_cmd, omega_cmd);
	Eigen::Matrix<double, 4, 1> commands;
	commands << 0, moments;
	return commands;

}

Eigen::Matrix<double, 12, 1> Controller::getState()
{
	return control_state;
}