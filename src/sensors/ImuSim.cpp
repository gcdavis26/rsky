#include "sensors/ImuSim.h"

ImuSim::ImuSim()
	: rng(std::random_device{}()),
	norm(0.0, 1.0)
{
}

void ImuSim::step(const Vec<12>& truth, double dt) {
	imu.gyro = truth.segment<3>(9) + gyro_bias;
	Vec<3> noise;
	noise << norm(rng), norm(rng), norm(rng);
	imu.gyro += gyro_noise_std * noise / sqrt(dt);

	Vec<3> a_ned = Vec<3>::Zero();
	if (!has_prev) {
		a_ned.setZero();
		has_prev = true;
		v_prev = truth.segment<3>(3);
	}
	else {
		a_ned = (truth.segment<3>(3) - v_prev) / dt;
		v_prev = truth.segment<3>(3);
	}
	Vec<3> g_ned;
	g_ned << 0.0, 0.0, g;
	
	Vec<3> f_b = RotB2N(truth(6), truth(7), truth(8)).transpose() * (a_ned - g_ned);
	
	imu.accel = f_b + accel_bias;
	noise << norm(rng), norm(rng), norm(rng);
	imu.accel += accel_noise_std * noise / sqrt(dt); 
}