cmake_minimum_required(VERSION 3.10)
project(NavioLibs)

set(common_folder ${CMAKE_CURRENT_SOURCE_DIR}/Common)
set(navio2_folder ${CMAKE_CURRENT_SOURCE_DIR}/Navio2)

set(Common_sources
${common_folder}/gpio.cpp
${common_folder}/I2Cdev.cpp
${common_folder}/MPU9250.cpp
${common_folder}/MS5611.cpp
${common_folder}/Ublox.cpp
${common_folder}/Util.cpp
)

set(Navio2_sources
${navio2_folder}/ADC_Navio2.cpp
${navio2_folder}/Led_Navio2.cpp
${navio2_folder}/LSM9DS1.cpp
${navio2_folder}/PWM.cpp
${navio2_folder}/RCInput_Navio2.cpp
${navio2_folder}/RCOutput_Navio2.cpp
${navio2_folder}/RGBled.cpp
)

add_library(Common STATIC ${Common_sources})

target_include_directories(Common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(Navio2 STATIC ${Navio2_sources})
target_include_directories(Navio2 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
	)
target_link_libraries(Navio2 PUBLIC Common)

