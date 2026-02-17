cmake_minimum_required(VERSION 3.16)

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
# Force executable output to root directory
# ==============================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# For multi-config generators (Visual Studio)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG_UPPER)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG_UPPER} ${CMAKE_SOURCE_DIR})
endforeach()

# ==============================
# Executable
# ==============================
add_executable(FireMission
    executables/main.cpp
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
    src/telemetry/udp_sender.cpp)

# ==============================
# Navio2 (Linux only)
# ==============================
if(UNIX AND NOT APPLE)
    target_compile_definitions(FireMission PRIVATE PLATFORM_LINUX)

    # Build Navio2 + Common static libs using Navio2's own CMakeLists.txt
    add_subdirectory(${CMAKE_SOURCE_DIR}/external/Navio2)

    # Your drivers
    target_sources(FireMission PRIVATE
        src/drivers/MotorDriver.cpp
        src/drivers/RCIn.cpp
		src/sensors/IMUHandler.cpp
    )

    # Link Navio2 (it PUBLIC-links Common)
    target_link_libraries(FireMission PRIVATE Navio2)
endif()

# ==============================
# Include directories
# ==============================
target_include_directories(FireMission PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/eigen
    ${CMAKE_SOURCE_DIR}/external/Navio2  
    ${CMAKE_SOURCE_DIR}/external

)

# ==============================
# Compiler warnings
# ==============================
if (MSVC)
    target_compile_options(FireMission PRIVATE /W4)
else()
    target_compile_options(FireMission PRIVATE -Wall -Wextra -Wpedantic)
endif()
