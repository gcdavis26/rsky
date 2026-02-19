#include "simulator/Dynamics.h"

Dynamics::Dynamics()
	: rng(std::random_device{}()),
	norm(0.0, 1.0)
{
	sigma_turb << 0.05, 0.05, 0.05;
}

Vec<12> Dynamics::getTrueState() {
	return state;
}

void Dynamics::step(double dt, const Vec<4>& bodyWrench) {

	Vec<3> pos = state.segment<3>(0);
	Vec<3> vel = state.segment<3>(3);

	Vec<3> euler = state.segment<3>(6);
	Vec<3> omega = state.segment<3>(9);

	const double Fz = bodyWrench(0); 
	const double Mx = bodyWrench(1);
	const double My = bodyWrench(2);
	const double Mz = bodyWrench(3);

	Mat<3,3> R_bn = RotB2N(euler(0), euler(1), euler(2));

	Vec<3> g_ned;
	g_ned << 0.0, 0.0, g;

	Vec<3> F_b;
	F_b << 0.0, 0.0, -Fz;

	Vec<3> a_ned = (1.0 / mass) * (R_bn * F_b) + g_ned;

	Vec<3> w;
	w << norm(rng), norm(rng), norm(rng);
	a_ned = a_ned + sigma_turb.cwiseProduct(w);

	Vec<3> euler_dot = eulerRates_ZYX(euler(0), euler(1), omega);

	const double p_dot = (Mx + (Iy - Iz) * omega(1) * omega(2)) / Ix;
	const double q_dot = (My + (Iz - Ix) * omega(0) * omega(2)) / Iy;
	const double r_dot = (Mz + (Ix - Iy) * omega(0) * omega(1)) / Iz;

	Vec<3> pos_dot;
	pos_dot << vel;
	double dragcoeff = 0.25;
	Vec<3> vel_dot = a_ned - dragcoeff * vel.norm() * vel;

	Vec<3> omega_dot;
	omega_dot << p_dot, q_dot, r_dot;

	state.segment<3>(0) = state.segment<3>(0) + pos_dot * dt;
	if (state(2) > 0) {
		state(2) = 0;
		state(5) = 0;
	}
	state.segment<3>(3) = state.segment<3>(3) + vel_dot * dt;

	state.segment<3>(6) = state.segment<3>(6) + euler_dot * dt;
	state.segment<3>(9) = state.segment<3>(9) + omega_dot * dt;

	state.segment<3>(6) = wrapAngles(state.segment<3>(6));

}