cmake_minimum_required(VERSION 3.25)
project(rsky)
set(CMAKE_CXX_STANDARD 17)

# --- Add Link Time Optimization (LTO / IPO) ---
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT error)
if(ipo_supported)
    message(STATUS "IPO / LTO is supported. Enabling globally.")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(WARNING "IPO / LTO is not supported: ${error}")
endif()

set(myExec ${CMAKE_SOURCE_DIR}/executables) 
set(extlibs ${CMAKE_SOURCE_DIR}/ext_libs)

add_subdirectory(${extlibs}/thermal-camera)
add_subdirectory(${extlibs}/Navio2)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

#add_executable(Simulation ${myExec}/simulation.cpp)
#target_compile_options(Simulation PRIVATE
#    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
#)
#target_include_directories(Simulation PRIVATE ${extlibs}/eigen)
#target_link_libraries(Simulation PRIVATE gnclib)
#target_precompile_headers(Simulation PRIVATE <Eigen/Dense>)

add_executable(RT_Simulation ${myExec}/rt_simulation.cpp)
target_compile_options(RT_Simulation PRIVATE
    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
)
target_include_directories(RT_Simulation PRIVATE ${extlibs}/eigen)
target_link_libraries(RT_Simulation PRIVATE gnclib drone_interface)
target_precompile_headers(RT_Simulation PRIVATE <Eigen/Dense>)


add_executable(drone ${myExec}/gain_testing.cpp)
target_include_directories(drone PRIVATE ${extlibs}/eigen)
target_link_libraries(drone PRIVATE gnclib drone_interface)
target_precompile_headers(RT_Simulation PRIVATE <Eigen/Dense>)

