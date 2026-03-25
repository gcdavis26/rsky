#include "common/LowPass.h"

ImuLpf::ImuLpf(float sample_rate_hz, float cutoff_hz)
	: sample_rate_hz_(sample_rate_hz), cutoff_hz_(cutoff_hz)
{
	compute_coefficients(sample_rate_hz, cutoff_hz);
	reset();
}

void ImuLpf::reset() {
	d1_.setZero();
	d2_.setZero();
	output_.setZero();
}

void ImuLpf::update(const Vecf<6>& raw)
{
    // Direct Form II Transposed
    output_ = b0_ * raw + d1_;
    d1_ = b1_ * raw - a1_ * output_ + d2_;
    d2_ = b2_ * raw - a2_ * output_;
}

void ImuLpf::compute_coefficients(float fs, float fc)
{
    const float K = std::tan(static_cast<float>(PI) * fc / fs);
    const float K2 = K * K;
    const float sqrt2 = std::sqrt(2.0f);
    const float norm = 1.0f + sqrt2 * K + K2;

    b0_ = K2 / norm;
    b1_ = 2.0f * K2 / norm;
    b2_ = K2 / norm;
    a1_ = 2.0f * (K2 - 1.0f) / norm;
    a2_ = (1.0f - sqrt2 * K + K2) / norm;
}