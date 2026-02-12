cmake_minimum_required(VERSION 3.5)
project(rsky)
set(CMAKE_CXX_STANDARD 17)

set(myExec ${CMAKE_SOURCE_DIR}/executables) 
set(extlibs ${CMAKE_SOURCE_DIR}/ext_libs)

add_subdirectory(${extlibs}/thermal-camera)
add_subdirectory(${extlibs}/Navio2)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

add_executable(simulation ${myExec}/pathing_simulation.cpp)
target_include_directories(simulation PUBLIC ${extlibs}/eigen)
target_link_libraries(simulation PRIVATE gnclib)

add_executable(drone ${myExec}/gain_testing.cpp)
target_include_directories(drone PUBLIC ${extlibs}/eigen)
target_link_libraries(drone PRIVATE gnclib drone_interface)


add_executable(throttle ${myExec}/throttle_pass.cpp)
target_link_libraries(throttle PRIVATE MLX90640_API Navio2)

add_executable(FTS ${myExec}/fts_test.cpp)
target_link_libraries(FTS PRIVATE drone_interface)