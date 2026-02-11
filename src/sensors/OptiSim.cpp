#include "sensors/OptiSim.h"

OptiSim::OptiSim()
	: rng(std::random_device{}()),
	norm(0.0, 1.0)
{
}
void OptiSim::step(const Vec<12>& truth) {
	opti.pos = truth.segment<3>(0);
	Vec<3> noise;
	noise << norm(rng), norm(rng), norm(rng);
	opti.pos += pos_noise_std * noise;

	opti.psi = truth(8);
	opti.psi += psi_noise_std * norm(rng);

	opti.psi = wrapToPi(opti.psi);
}