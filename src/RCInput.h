#pragma once

#include <cstddef>
#include <Eigen/Dense>
#include <Navio2/RCInput_Navio2.h>

class RCInput_Navio2 : public RCInput
{
public:
    // Constructor handles all file opening
    RCInput_Navio2();

    // These must match the signatures in Common/RCInput.h exactly
    void initialize() override;
    int read(int ch) override;

    // Your custom matrix function for the FlySky receiver
    Eigen::Matrix<double, 6, 1> read_ppm_vector();

private:
    int open_channel(int ch);

    static const size_t CHANNEL_COUNT = 14;
    int channels[CHANNEL_COUNT];
};