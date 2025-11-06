add_library(Navio2 STATIC 
ADC_Navio2.cpp
Led_Navio2.cpp
LSM9DS1.cpp
PWM.cpp
RCInput_Navio2.cpp
RCOutput_Navio2.cpp
RGBled.cpp
)
target_include_directories(Navio2 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)