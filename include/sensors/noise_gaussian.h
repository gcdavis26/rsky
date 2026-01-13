#pragma once

#include <random>

namespace gnc {

class GaussianNoise {
public:
  GaussianNoise()
    : rng_(std::random_device{}()),
      dist_(0.0, 1.0) {}

  explicit GaussianNoise(uint32_t seed)
    : rng_(seed),
      dist_(0.0, 1.0) {}

  // Standard normal N(0,1)
  double n01() { return dist_(rng_); }

  // Normal N(0, sigma^2)
  double n(double sigma) { return sigma * dist_(rng_); }

private:
  std::mt19937 rng_;
  std::normal_distribution<double> dist_;
};

} // namespace gnc
