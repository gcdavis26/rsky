#pragma once

#include "common/MathUtils.h"

class AHRS {
public:
    AHRS();
	AHRS(const Vec<6>& bias);

	void initializeFromAccel(const Vec<3>& accel); 
	void initialize(double roll, double pitch);
	void update(const Vec<3>& accel,const Vec<3>& gyro, double dt);

	Vec<3> euler() const { return Vec<3>(phi, theta, psi); }

	bool init = false;

private:
    void accelToAttitude(const Vec<3>& accel, double& roll, double& pitch);

    Vec<6> AHRSbias;
    double phi = 0.0, theta = 0.0, psi = 0.0;

    Vec<4> x = (Vec<4>() << 0, 0, 0, 0).finished();
    Mat<4,4> P = Mat<4,4>::Identity();


    double q_angle = 0.01; // gyro noise gx,gy
    double q_bias = 1e-6;   //bias walk;
    double r_tilt = 0.01; // (sigma_acc/g)^2

    double gate_g = 0.15;     
    double max_abs_pitch_rad = 1.50; 
};