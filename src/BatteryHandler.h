#ifndef BATTERYHANDLER_H
#define BATTERYHANDLER_H

#include "Navio2/ADC_Navio2.h"
#include <Eigen/Dense>

class BatteryHandler {
public:
    BatteryHandler();

    // Returns 2x1 Eigen Matrix [Voltage (mV), Current (mV)]
    Eigen::Vector2d read_battery();

    // ADC channels (0-5 on Navio2)
    // 0: Board voltage
    // 1: Servo rail voltage
    // 2: Power module voltage
    // 3: Power module current
    // 4: ADC port (AIN1)
    // 5: ADC port (AIN2)

private:
    ADC_Navio2 adc;
};

#endif