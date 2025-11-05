#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

// Set desired rate in Hz (Options: 1, 2, 4, 8, 16, 32, 64)
#define DESIRED_REFRESH_RATE 8

#define MLX90640_ADDRESS 0x33
#define FRAME_SIZE 834  // frame data size
#define WIDTH 32
#define HEIGHT 24

// Converts a desired refresh rate in Hz to the camera's register value.
uint16_t get_refresh_rate_value(int rate_hz) {
    uint16_t rate_value;
    switch (rate_hz) {
    case 1:  rate_value = 0x01; break; // 1 Hz
    case 2:  rate_value = 0x02; break; // 2 Hz
    case 4:  rate_value = 0x03; break; // 4 Hz
    case 8:  rate_value = 0x04; break; // 8 Hz
    case 16: rate_value = 0x05; break; // 16 Hz
    case 32: rate_value = 0x06; break; // 32 Hz
    case 64: rate_value = 0x07; break; // 64 Hz
    }
    return rate_value;
}

int main() {
    // Get the register value for the camera from the desired rate
    uint16_t camera_rate_value = get_refresh_rate_value(DESIRED_REFRESH_RATE);

    // Calculate the required loop delay (1,000,000 microseconds in a second)
    const int usleep_delay = 1000000 / DESIRED_REFRESH_RATE;

    const char* i2c_device = "/dev/i2c-1";
    int fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C device." << std::endl;
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, MLX90640_ADDRESS) < 0) {
        std::cerr << "Failed to set I2C address." << std::endl;
        close(fd);
        return -1;
    }

    // Read EEPROM data for calibration
    uint16_t eeMLX90640[832];
    if (MLX90640_DumpEE(MLX90640_ADDRESS, eeMLX90640) != 0) {
        std::cerr << "Failed to read EEPROM." << std::endl;
        close(fd);
        return -1;
    }

    paramsMLX90640 mlx90640;
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    // Set the camera's refresh rate
    MLX90640_SetRefreshRate(MLX90640_ADDRESS, camera_rate_value);

    float emissivity = 0.95;
    uint16_t frameData[FRAME_SIZE];
    float frame[WIDTH * HEIGHT];

    while (true) {
        if (MLX90640_GetFrameData(MLX90640_ADDRESS, frameData) != 0) {
            std::cerr << "Failed to get frame data." << std::endl;
            continue;
        }

        float tr = MLX90640_GetTa(frameData, &mlx90640) - 8.0;
        MLX90640_CalculateTo(frameData, &mlx90640, emissivity, tr, frame);

        // Map temperature to ASCII characters for visualization
        const char* levels = " .:-=+*#%@";  // we still reuse this palette
        const char mids[3] = { ':', '*', '#' }; // exactly 3 mid-range levels

        for (int y = 0; y < HEIGHT; ++y) {
            for (int x = 0; x < WIDTH; ++x) {
                float temp = frame[y * WIDTH + x];
                char outChar;

                if (temp < 30.0f) {
                    outChar = ' '; // Low Pass Filter removing all sub 30C values
                }
                else if (temp < 40.0f) {
                    // Split [30,40) into 3 equal bins of ~3.3333°C
                    const float bin_width = (40.0f - 30.0f) / 3.0f; // ≈ 3.3333
                    int idx = static_cast<int>((temp - 30.0f) / bin_width);
                    // manual bounds (no clamp)
                    if (idx < 0) idx = 0;
                    if (idx > 2) idx = 2;
                    outChar = mids[idx];
                }
                else {
                    outChar = levels[9]; // '@' (strongest) for >= 40
                }

                std::cout << outChar;
            }
            std::cout << "\n";
        }

        // Pause the loop to match the camera's sampling rate
        usleep(usleep_delay);
    }

    close(fd);
    return 0;
}