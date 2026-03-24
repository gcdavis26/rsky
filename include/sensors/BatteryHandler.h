#pragma once

#include <Eigen/Dense>

#ifdef PLATFORM_LINUX
#include <Navio2/ADC_Navio2.h>
#endif

class BatteryHandler {
public:
    BatteryHandler();
    Eigen::Vector2d read_battery();

private:
#ifdef PLATFORM_LINUX
    ADC_Navio2 adc;
#endif
};
