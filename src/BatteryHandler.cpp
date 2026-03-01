#include "BatteryHandler.h"

BatteryHandler::BatteryHandler() {
    adc.initialize();
}

Eigen::Vector2d BatteryHandler::read_battery() {
    Eigen::Vector2d battery_data;

    // Read the raw values (returns millivolts from the ADC)
    int voltage_mv = adc.read(2);
    int current_mv = adc.read(3);

    // Note: You may need to multiply these by your power module's specific 
    // voltage/current scaling multipliers to get actual Volts and Amps.
    battery_data(0) = static_cast<double>(voltage_mv);
    battery_data(1) = static_cast<double>(current_mv);

    return battery_data;
}

int BatteryHandler::read_channel(int channel) {
    return adc.read(channel);
}