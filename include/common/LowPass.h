#pragma once
#include "common/MathUtils.h"
#include <Eigen/Dense>
#include <cmath>

class ImuLpf {
public:
    ImuLpf(float sample_rate_hz, float cutoff_hz);

    void reset();

    void update(const Vec<6>& raw);

    const Vec<6>& output() const { 
        if (on) {
            return output_;
        }
        else {
            return unfiltered;
        }
    }
   
    float cutoff_hz()      const { return cutoff_hz_; }
    float sample_rate_hz() const { return sample_rate_hz_; }

    bool on = true;

private:
    void compute_coefficients(float fs, float fc);

    float b0_, b1_, b2_, a1_, a2_;
    Vec<6> d1_, d2_, output_;
    Vec<6> unfiltered;
    float sample_rate_hz_, cutoff_hz_;
};
