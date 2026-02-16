cmake_minimum_required(VERSION 3.5)
project(rsky)
set(CMAKE_CXX_STANDARD 17)

set(myExec ${CMAKE_SOURCE_DIR}/executables) 
set(extlibs ${CMAKE_SOURCE_DIR}/ext_libs)

add_subdirectory(${extlibs}/thermal-camera)
add_subdirectory(${extlibs}/Navio2)
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

add_executable(Simulation ${myExec}/simulation.cpp)
target_compile_options(Simulation PRIVATE
    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
)
target_include_directories(Simulation PRIVATE ${extlibs}/eigen)
target_link_libraries(Simulation PRIVATE gnclib)

add_executable(RT_Simulation ${myExec}/rt_simulation.cpp)
target_compile_options(RT_Simulation PRIVATE
    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
)
target_include_directories(RT_Simulation PRIVATE ${extlibs}/eigen)
target_link_libraries(RT_Simulation PRIVATE gnclib drone_interface)


add_executable(drone ${myExec}/gain_testing.cpp)
target_include_directories(drone PRIVATE ${extlibs}/eigen)
target_link_libraries(drone PRIVATE gnclib drone_interface)

