#pragma once

#include "common/types.h"

namespace gnc {

class MotorModelFirstOrder {
public:
  // tau: time constant [s] (bigger = slower motors)
  MotorModelFirstOrder(double tau_sec);

  // Step motor model: returns updated thrusts [N]
  Vec4 step(double dt, const Vec4& thrust_cmd);

  // Set/Reset
  void resetTo(const Vec4& thrust_initial);

  Vec4 thrust() const;

private:
  double tau_;
  Vec4 thrust_actual_;
};

} // namespace gnc
