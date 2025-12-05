#pragma once
#include <Eigen/Dense>
class Controller //NED controller
{
private:
	Eigen::Matrix<double, 12, 1> control_state; //different from ekf, but derivable from ekf. Pos, vel, euler angles, body rates
	Eigen::Vector3d pos_Kp; //pos
	Eigen::Vector3d pos_Kd; //vel
	Eigen::Vector3d att_Kp; //attitude
	Eigen::Vector3d att_Kd; //angular rates
	std::pair<double, double> T_sat;
	std::pair<double, double> a_sat;
	double max_angle;
	Eigen::Vector3d outer_achievePos(Eigen::Vector3d p_des,Eigen::Vector3d v_des); //returns att + T
	Eigen::Vector3d inner_achieveAtt(Eigen::Vector3d att_des, Eigen::Vector3d omega_des); //returns L M N
	Eigen::Vector3d g;
	double m;
public:
	Controller::Controller(Eigen::Vector3d outer_Kp, Eigen::Vector3d outer_Kd, Eigen::Vector3d inner_Kp, Eigen::Vector3d inner_Kd, std::pair<double, double> thrust_sat,
		std::pair<double, double> acc_sat, double angle_maximum,double m);
	void Controller::update(Eigen::Matrix<double, 15, 1> x, Eigen::Vector3d omega); //This stuff comes from the ekf
	void Controller::update(Eigen::Matrix<double, 12, 1> x); //This is from no ekf simulation
	Eigen::Matrix<double, 4, 1> Controller::achieveState(Eigen::Vector3d p_des, Eigen::Vector3d v_des, double psi_des, Eigen::Vector3d omega_des); //returns T L M N
	Eigen::Matrix<double, 12, 1> Controller::getState();

};