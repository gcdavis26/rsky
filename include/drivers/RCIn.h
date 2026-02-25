#pragma once

#include <Eigen/Dense>
#include "Navio2/RCInput_Navio2.h"

// High-level FlySky / control mapping wrapper
class RCIn final : public RCInput_Navio2
{
public:
    RCIn() = default;
    ~RCIn() = default;

    // Optional: only if you want extra init behavior
    void initialize();

    // Your added functionality
    Eigen::Matrix<double, 6, 1> read_ppm_vector();
};
