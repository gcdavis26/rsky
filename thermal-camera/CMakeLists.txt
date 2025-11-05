cmake_minimum_required(VERSION 3.10)
project(mlx90640_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define source files for the MLX90640 library
set(MLX90640_API_SOURCES
    ./MLX90640_API.c
    ./MLX90640_I2C_Driver.cpp
)

# Create MLX90640 API static library
add_library(MLX90640_API STATIC ${MLX90640_API_SOURCES})

# Add include directories for MLX90640_API
target_include_directories(MLX90640_API PUBLIC
    ${CMAKE_SOURCE_DIR}
)

# Example executable using MLX90640

# Example executable using MLX90640
add_executable(mlx90640_example ./main.cpp)
target_include_directories(mlx90640_example PUBLIC
    ${CMAKE_SOURCE_DIR}
)

# Link libraries to the example executable
target_link_libraries(mlx90640_example
    PRIVATE
    MLX90640_API
    pthread
    rt
)

# Compiler flags for optimization and debugging
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    execute_process(COMMAND nproc OUTPUT_VARIABLE NPROC OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -Wextra -g -fno-omit-frame-pointer -O0")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /arch:AVX2")
endif()

# Enable parallel link-time optimization
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    execute_process(COMMAND nproc OUTPUT_VARIABLE NPROC OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CMAKE_JOB_POOLS link=${NPROC})
    set(CMAKE_JOB_POOL_LINK link)
endif()