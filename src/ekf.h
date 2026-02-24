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
	Eigen::Matrix<double, 15, 12> G;
	Matrix12d Q;
	Matrix4d R;
	Vector3d g;
	Vector3d radius;
	Vector3d alpha;
	Vector3d body_accels;
	Vector3d omega_measured;
	Eigen::Matrix<double, 15, 4> K;
public:
	//Constructor. Need gravity vector in NED, 
	EKF(const Vector3d& r, const Vector12d& sigmaw, const Vector4d& sigmav);
	void initialize(const Vector4d& measurement, const Vector3d& gyro0, const Vector3d& accel0, const Vector3d& bias_accel, const Vector3d& bias_gyro);//set up initial states. Initial measurement, per se
	void imureading(const Vector3d& omega, const Vector3d& new_imu_accels, double dt);
	void estimate(double dt);
	void update(const Vector4d& m);
	Vector15d getState();
	Vector12d getControlState();
	Vector3d getOmega();
};

#endif