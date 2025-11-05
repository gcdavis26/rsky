#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define MLX90640_ADDRESS 0x33
#define FRAME_SIZE 834  // frame data size
#define WIDTH 32
#define HEIGHT 24

int main() {
    const char *i2c_device = "/dev/i2c-1";
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
    MLX90640_SetRefreshRate(MLX90640_ADDRESS, 0x03);  // 4 Hz

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
                    outChar = '.'; // explicit low marker
                } else if (temp < 40.0f) {
                    // Split [30,40) into 3 equal bins of ~3.3333°C
                    const float bin_width = (40.0f - 30.0f) / 3.0f; // ≈ 3.3333
                    int idx = static_cast<int>((temp - 30.0f) / bin_width);
                    // manual bounds (no clamp)
                    if (idx < 0) idx = 0;
                    if (idx > 2) idx = 2;
                    outChar = mids[idx];
                } else {
                    outChar = levels[9]; // '@' (strongest) for >= 40
                }

                std::cout << outChar;
            }
            std::cout << "\n";
        }
    usleep(500000);  // 2Hz sampling rate (500ms)
    }

    close(fd);
    return 0;
}
