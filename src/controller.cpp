#include <Eigen/Dense>
#include "controller.h"
#include "math_utils.h"
#include <cmath>
#include <algorithm>
Controller::Controller(Vector3d outer_Kp, Vector3d outer_Kd, Vector3d inner_Kp, Vector3d inner_Kd, std::pair<double, double> thrust_sat,
	std::pair<double, double> acc_sat, double angle_maximum, double mass)
{
	pos_Kp = outer_Kp;
	pos_Kd = outer_Kd;
	att_Kp = inner_Kp;
	att_Kd = inner_Kd;
	T_sat = thrust_sat;//min,max
	a_sat = acc_sat; //max xy, max z
	max_angle = angle_maximum;
	control_state = Vector12d::Zero(); //att, body_rate, pos, vel
	g << 0, 0, 9.81;
	m = mass;
}

void Controller::update(Vector12d x) 
{
	control_state = x;
}


Vector3d Controller::outer_achievePos(Vector3d p_des, Vector3d v_des)
{
	//outer loop
	Vector3d e_pos = p_des - control_state.block(6, 0, 3, 1);
	Vector3d e_vel = v_des - control_state.block(9, 0, 3, 1);
	Vector3d a_cmd = e_pos.array() * pos_Kp.array() + e_vel.array() * pos_Kd.array();
	a_cmd(0) = saturate(a_cmd(0), a_sat.first);
	a_cmd(1) = saturate(a_cmd(1), a_sat.first);
	a_cmd(2) = saturate(a_cmd(2), a_sat.second);
	double T_cmd = (g(2) - a_cmd(2)) * m;
	T_cmd = saturate(T_cmd, T_sat.first, false);//making sure greater than min thrust
	T_cmd = saturate(T_cmd, T_sat.second); //less than max thrust
	Matrix2d A;
	A << sin(control_state(2)),cos(control_state(2)),-cos(control_state(2)),sin(control_state(2));
	Vector2d res =-m / T_cmd * A.transpose()*a_cmd.block(0,0,2,1); //A inverse = A transpose for this specific matrix

	double phi_cmd = saturate(res(0), max_angle);
	double theta_cmd = saturate(res(1),max_angle);
	
	Vector3d output;
	output << phi_cmd, theta_cmd, T_cmd;
	return output;
}

Vector3d Controller::inner_achieveAtt(Vector3d att_cmd, Vector3d omega_cmd)
{
	Vector3d e_att = att_cmd - control_state.block(0, 0, 3, 1);
	Vector3d e_omega = omega_cmd - control_state.block(3, 0, 3, 1);
	Vector3d moments = e_att.array() * att_Kp.array() + e_omega.array() * att_Kd.array();
	return moments;
}

Vector4d Controller::achieveState(double psi_cmd, Vector3d omega_cmd, Vector3d p_cmd, Vector3d v_cmd) //returns T L M N
{
	Vector3d outer_output = outer_achievePos(p_cmd, v_cmd);
	Vector3d att_cmd;
	att_cmd << outer_output(0), outer_output(1), psi_cmd;
	Vector3d moments = inner_achieveAtt(att_cmd, omega_cmd);
	Vector4d commands;
	commands << outer_output(2), moments;
	return commands;
}

Vector4d Controller::innerTest(Vector3d att_cmd, Vector3d omega_cmd) // for inner loop control only: For test stand and manual
{
	Vector3d moments = inner_achieveAtt(att_cmd, omega_cmd);
	Vector4d commands;
	commands << 0, moments;
	return commands;

}

Vector12d Controller::getState()
{
	return control_state;
}