cmake_minimum_required(VERSION 3.5)
project(rsky)
set(CMAKE_CXX_STANDARD 14)

set(myExec ${CMAKE_SOURCE_DIR}/executables) 
set(extlibs ${CMAKE_SOURCE_DIR}/ext_libs)

add_subdirectory(${extlibs}/thermal-camera)
add_subdirectory(${extlibs}/Navio2)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

add_executable(simulation ${myExec}/simulation.cpp)
target_include_directories(simulation PUBLIC ${extlibs}/eigen)
target_link_libraries(simulation PRIVATE gnclib)
add_executable(throttle ${myExec}/throttle_pass.cpp)
target_link_libraries(throttle PRIVATE thermal-camera Common Navio2)
