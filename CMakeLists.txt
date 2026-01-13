cmake_minimum_required(VERSION 3.16)

# Project name and language
project(QuadcopterGNC LANGUAGES CXX)

# ==============================
# C++ standard
# ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================
# Executable
# ==============================
add_executable(sim_main
    executables/sim_main.cpp
	src/sim/logger.cpp
	src/common/math_util.cpp
	src/plant/quad_dynamics_euler.cpp
	src/control/quad_mixer_x.cpp
	src/plant/motor_model_first_order.cpp
	src/control/inner_loop_attitude_pd.cpp
	src/control/outer_loop_position_pd.cpp
	src/control/coverage_vector_field.cpp
	src/control/outer_loop_coverage.cpp
	src/guidance/mode_manager.cpp
	src/sensors/imu_sim.cpp
	src/sensors/optitrack_sim.cpp
	src/estimation/ekf15_full.cpp
)

# ==============================
# Include directories
# ==============================
# - include/: your headers
# - external/eigen/: must contain Eigen/ directory
target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/eigen
)

# ==============================
# Compiler warnings (safe defaults)
# ==============================
if (MSVC)
    target_compile_options(sim_main PRIVATE /W4)
else()
    target_compile_options(sim_main PRIVATE -Wall -Wextra -Wpedantic)
endif()

