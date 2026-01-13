#include "sensors/optitrack_sim.h"

#include <cmath>

namespace gnc {

OptiTrackSim::OptiTrackSim(const OptiSimParams& p, uint32_t seed)
  : p_(p),
    noise_(seed) {}

OptiMeas OptiTrackSim::step(const QuadStateTruth& truth) {
  OptiMeas m;
  m.t = truth.t;

  m.pos_ned = truth.pos;
  m.pos_ned(0) += noise_.n(p_.pos_noise_std);
  m.pos_ned(1) += noise_.n(p_.pos_noise_std);
  m.pos_ned(2) += noise_.n(p_.pos_noise_std);

  m.psi = wrapToPi(truth.euler(2) + noise_.n(p_.psi_noise_std));
  return m;
}

double OptiTrackSim::wrapToPi(double a) {
  while (a >  PI) a -= 2.0 * PI;
  while (a < PI) a += 2.0 * PI;
  return a;
}

} // namespace gnc
