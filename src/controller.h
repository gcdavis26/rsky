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
	Vector3d outer_achievePos(const Vector3d& p_des, const Vector3d& v_des); //returns att + T
	Vector3d inner_achieveAtt(const Vector3d& att_cmd, const Vector3d& omega_cmd); //returns L M N
	Vector3d g;
	double m;
public:
	Controller(const Vector3d& outer_Kp, const Vector3d& outer_Kd, const Vector3d& inner_Kp, const Vector3d& inner_Kd, std::pair<double, double> thrust_sat,
		std::pair<double, double> acc_sat, double angle_maximum, double mass);
	void update(const Vector12d& x);
	Vector4d achieveState(double psi_cmd, const Vector3d& omega_cmd, const Vector3d& p_cmd, const Vector3d& v_cmd); //returns T L M N
	Vector4d manualControl(const Vector4d& cmds);
	Vector12d getState();

};