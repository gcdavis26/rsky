#include "plant/motor_model_first_order.h"

namespace gnc {

MotorModelFirstOrder::MotorModelFirstOrder(double tau_sec)
  : tau_(tau_sec)
{
  thrust_actual_.setZero();
}

void MotorModelFirstOrder::resetTo(const Vec4& thrust_initial) {
  thrust_actual_ = thrust_initial;
}

Vec4 MotorModelFirstOrder::thrust() const {
  return thrust_actual_;
}

Vec4 MotorModelFirstOrder::step(double dt, const Vec4& thrust_cmd) {
  // If tau is tiny, behave like "no delay"
  if (tau_ <= 1e-9) {
    thrust_actual_ = thrust_cmd;
    return thrust_actual_;
  }

  // First-order lag
  double alpha = dt / tau_;
  if (alpha > 1.0) {
    alpha = 1.0; // keep stable if dt > tau
  }

  thrust_actual_ = thrust_actual_ + alpha * (thrust_cmd - thrust_actual_);
  return thrust_actual_;
}

} // namespace gnc
