#ifndef EKF_H
#define EKF_H
#include <Eigen/Dense>
#include "math_utils.h"
class EKF
{
private:
	Vector15d x;
	Matrix15d P;
	Matrix15d A;
	Matrix12d Q;
	Matrix3d R;
	Vector3d g;
	Vector3d alpha;
	Vector3d body_accels;
	Vector3d radius;
	Vector3d omega_measured;
	double dt;

public:
	//Constructor. Need gravity vector in NED, 
	EKF::EKF(Vector3d r, Vector12d sigmaw, Vector3d sigmav, double freq);
	void EKF::initialize(Vector3d measurement, Vector3d gyro0, Vector3d accel0, Vector3d bias_accel, Vector3d bias_gyro); //set up initial states. Initial measurement, per se
	void EKF::imureading(Vector3d omega, Vector3d new_imu_accels);
	void EKF::estimate();
	void EKF::update(Vector3d m);
	Vector15d getState();
	Vector12d getControlState();
	Vector3d getOmega();
};

#endif