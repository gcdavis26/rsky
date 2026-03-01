#include "BatteryHandler.h"

BatteryHandler::BatteryHandler() {
    adc.initialize();
}

Eigen::Vector2d BatteryHandler::read_battery() {
    Eigen::Vector2d battery_data;

    // Read raw ADC millivolts
    double raw_voltage_mv = adc.read(2);
    double raw_current_mv = adc.read(3);

    // Apply calibration scaling
    // Using constants used in Navio2 SAS
    double actual_voltage_mv = raw_voltage_mv * 10.88;
    double actual_current_ma = raw_current_mv * 200.0;

    battery_data(0) = actual_voltage_mv;
    battery_data(1) = actual_current_ma;

    return battery_data;
}
