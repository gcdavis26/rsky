cmake_minimum_required(VERSION 3.10)
project(mlx90640_driver)

# Define source files for the MLX90640 library
set(MLX90640_API_SOURCES
    ./MLX90640_API.c
    ./MLX90640_I2C_Driver.cpp
)

# Create MLX90640 API static library
add_library(MLX90640_API STATIC ${MLX90640_API_SOURCES})

# Expose headers so the drone's vision algorithms can #include them
target_include_directories(MLX90640_API PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# MLX90640_API.c heavily uses pow(), sqrt(), etc. 
# Linux requires linking 'm' for these C-math functions.
target_link_libraries(MLX90640_API PRIVATE m)