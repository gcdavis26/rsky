add_library(Common STATIC 
gpio.cpp
I2Cdev.cpp
MPU9250.cpp
MS5611.cpp
Ublox.cpp
Util.cpp
)
target_include_directories(Common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)