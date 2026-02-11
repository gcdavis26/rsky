#include "drivers/RCIn.h"

void RCIn::initialize() {
    RCInput_Navio2::initialize();
}

Eigen::Matrix<double, 6, 1> RCIn::read_ppm_vector()
{
    Eigen::Matrix<double, 6, 1> pwm_matrix;

    // Channel 0: v_east
    // Channel 1: v_north
    // Channel 2: Throttle. Power
    // Channel 3: Rudder.
    // Channel 4: Aux Channel
    // Channel 5: Aux Channel

    for (int i = 0; i < 6; ++i) {
        int val = this->read(i);
        pwm_matrix(i) = val;
    }

    return pwm_matrix;
}
