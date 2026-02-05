#pragma once
#include <Eigen/Dense>
#include "math_utils.h"
class Controller //NED controller
{
private:
	Vector12d control_state; //different from ekf, but derivable from ekf. Pos, vel, euler angles, body rates
	Vector3d pos_Kp; //pos
	Vector3d pos_Kd; //vel
	Vector3d att_Kp; //attitude
	Vector3d att_Kd; //angular rates
	std::pair<double, double> T_sat;
	std::pair<double, double> a_sat;
	double max_angle;
	Vector3d outer_achievePos(Vector3d p_des,Vector3d v_des); //returns att + T
	Vector3d inner_achieveAtt(Vector3d att_des, Vector3d omega_des); //returns L M N
	Vector3d g;
	double m;
public:
	Controller::Controller(Vector3d outer_Kp, Vector3d outer_Kd, Vector3d inner_Kp, Vector3d inner_Kd, std::pair<double, double> thrust_sat,
		std::pair<double, double> acc_sat, double angle_maximum,double m);
	void Controller::update(Vector12d x); //This is from no ekf simulation
	Vector4d Controller::achieveState(double psi_cmd, Vector3d omega_cmd, Vector3d p_cmd, Vector3d v_cmd); //returns T L M N
	Vector4d Controller::innerTest(Vector3d att_cmd, Vector3d omega_cmd);
	Vector12d Controller::getState();

};