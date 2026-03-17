#pragma once

#include <array>
#include "MLX90640_API.h"

class ThermalCamera {
public:
    // Defaults to the standard MLX90640 I2C address and Raspberry Pi I2C bus 1
    ThermalCamera(int address = 0x33, int bus = 1);
    ~ThermalCamera();

    // Opens the I2C bus, extracts EEPROM, and sets the 32Hz refresh rate
    bool init();

    // Fetches the latest frame and calculates temperatures into the provided buffer
    bool getFrame(std::array<double, 768>& buffer);

private:
    int i2c_addr;
    int i2c_bus;
    int fd;

    double emissivity;
    paramsMLX90640 mlx90640;
    uint16_t frameData[834]; // Internal buffer required by the MLX API
};