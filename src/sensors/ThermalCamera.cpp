#include "ThermalCamera.h"
#include "MLX90640_I2C_Driver.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string>

ThermalCamera::ThermalCamera(int address, int bus)
    : i2c_addr(address), i2c_bus(bus), fd(-1), emissivity(0.95) {
    init();
}

ThermalCamera::~ThermalCamera() {
    if (fd >= 0) {
        close(fd);
    }
}

bool ThermalCamera::init() {
    std::string device = "/dev/i2c-" + std::to_string(i2c_bus);
    fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C device: " << device << std::endl;
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        std::cerr << "Failed to set I2C address." << std::endl;
        close(fd);
        fd = -1;
        return false;
    }

    // Read EEPROM data for calibration
    uint16_t eeMLX90640[832];
    if (MLX90640_DumpEE(i2c_addr, eeMLX90640) != 0) {
        std::cerr << "Failed to read EEPROM." << std::endl;
        return false;
    }

    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    // Set to 32 Hz (0x06)
    MLX90640_SetRefreshRate(i2c_addr, 0x06);

    return true;
}

bool ThermalCamera::getFrame(std::array<double, 768>& buffer) {
    if (fd < 0){
 	return false;
    }

    // Fetch the raw data
    if (MLX90640_GetFrameData(i2c_addr, frameData) <  0) {
	std::cout << MLX90640_GetFrameData(i2c_addr, frameData) << "\n";
	return false;
    }

    // Calculate reflected temperature and convert to absolute temperatures
    double tr = MLX90640_GetTa(frameData, &mlx90640) - 8.0;

    // MLX API fills the buffer array with double precision temperatures
    MLX90640_CalculateTo(frameData, &mlx90640, emissivity, tr, buffer.data());

    return true;
}
