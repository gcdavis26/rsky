cmake_minimum_required(VERSION 3.25)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT)
endif()

project(QuadcopterGNC LANGUAGES CXX)

# ==============================
# C++ standard
# ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================
# Link Time Optimization (IPO / LTO)
# ==============================
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT error)
if(ipo_supported)
    message(STATUS "IPO / LTO is supported. Enabling globally.")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(WARNING "IPO / LTO is not supported: ${error}")
endif()

# ==============================
# Force executable output to root directory
# ==============================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# For multi-config generators (Visual Studio)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_SOURCE_DIR})
endforeach()

# ==============================
# Shared source files
# ==============================
set(COMMON_SOURCES
    src/common/TimeKeeper.cpp
    src/common/MathUtils.cpp
    src/control/InnerLoop.cpp
    src/control/OuterLoop.cpp
    src/guidance/ModeManager.cpp
    src/estimation/EKF.cpp
    src/control/QuadMixer.cpp
    src/simulator/Dynamics.cpp
    src/simulator/MotorModel.cpp
    src/sensors/ImuSim.cpp
    src/sensors/OptiSim.cpp
    src/telemetry/udp_sender.cpp
    src/telemetry/TelemetryTask.cpp
    src/sensors/BatteryHandler.cpp
    src/estimation/AHRS.cpp
    src/common/LowPass.cpp
    src/drivers/MotorTask.cpp
)

# ==============================
# Executables
# ==============================

if(UNIX AND NOT APPLE)
    add_executable(FireM2
        executables/main_no_timekeeper.cpp
        ${COMMON_SOURCES}
    )

    add_executable(ThermalTest
        executables/Camera_test.cpp
        ${COMMON_SOURCES}
    )
endif()

# ==============================
# Shared include directories
# ==============================
if(UNIX AND NOT APPLE)
target_include_directories(FireM2 PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/sensors
    ${CMAKE_SOURCE_DIR}/include/vision
    ${CMAKE_SOURCE_DIR}/external/eigen
    ${CMAKE_SOURCE_DIR}/external/Navio2
    ${CMAKE_SOURCE_DIR}/external
)

target_include_directories(ThermalTest PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/sensors
    ${CMAKE_SOURCE_DIR}/include/vision
    ${CMAKE_SOURCE_DIR}/external/eigen
    ${CMAKE_SOURCE_DIR}/external/Navio2
    ${CMAKE_SOURCE_DIR}/external
)

endif()


# ==============================
# Precompile Headers
# ==============================
if (UNIX AND NOT APPLE)
target_precompile_headers(FireM2 PRIVATE <Eigen/Dense>)
target_precompile_headers(ThermalTest PRIVATE <Eigen/Dense>)
endif()

# ==============================
# Compiler warnings & Optimizations
# ==============================
if (MSVC)
    if (UNIX AND NOT APPLE)
    target_compile_options(FireM2 PRIVATE /W4)
    target_compile_options(ThermalTest PRIVATE /W4)
    endif()

    # MSVC Release Optimizations
    if (UNIX AND NOT APPLE)
    target_compile_options(FireM2 PRIVATE $<$<CONFIG:Release>:/O2 /Ob2 /arch:AVX2 /DNDEBUG>)
    target_compile_options(ThermalTest PRIVATE $<$<CONFIG:Release>:/O2 /Ob2 /arch:AVX2 /DNDEBUG>)
    endif()
else()
    if (UNIX AND NOT APPLE)
    target_compile_options(FireM2 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ThermalTest PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    # GCC/Clang Release Optimizations
    if (UNIX AND NOT APPLE)
    target_compile_options(FireM2 PRIVATE $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>)
    target_compile_options(ThermalTest PRIVATE $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>)
    endif()
    
endif()

# ==============================
# Navio2 & Hardware (Linux only)
# ==============================
if(UNIX AND NOT APPLE)
    target_compile_definitions(FireM2 PRIVATE PLATFORM_LINUX)
    target_compile_definitions(ThermalTest PRIVATE PLATFORM_LINUX)

    # Build Navio2 + Common static libs using Navio2's own CMakeLists.txt
    add_subdirectory(${CMAKE_SOURCE_DIR}/external/Navio2)

    # Build the Melexis MLX90640 raw I2C drivers
    add_subdirectory(${CMAKE_SOURCE_DIR}/external/thermal-camera)

    target_sources(FireM2 PRIVATE
        src/drivers/MotorDriver.cpp
        src/drivers/RCIn.cpp
        src/sensors/IMUHandler.cpp
        src/mocap/network.cpp
        src/mocap/mocapHandler.cpp
        src/sensors/ThermalCamera.cpp
        src/vision/SearchGrid.cpp
        src/vision/WildfireDetection.cpp
    )

    target_sources(ThermalTest PRIVATE
        src/drivers/MotorDriver.cpp
        src/drivers/RCIn.cpp
        src/sensors/IMUHandler.cpp
        src/mocap/network.cpp
        src/mocap/mocapHandler.cpp
        src/sensors/ThermalCamera.cpp
        src/vision/SearchGrid.cpp
        src/vision/WildfireDetection.cpp
    )

    # Link Navio2 and the MLX library
    target_link_libraries(FireM2 PRIVATE Navio2 MLX90640_API)
    target_link_libraries(ThermalTest PRIVATE Navio2 MLX90640_API)
endif()

