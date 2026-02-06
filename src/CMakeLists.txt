project(src)
set(gnc_sources 
${CMAKE_CURRENT_SOURCE_DIR}/math_utils.cpp
${CMAKE_CURRENT_SOURCE_DIR}/ekf.cpp
${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp
${CMAKE_CURRENT_SOURCE_DIR}/guidance.cpp
)
add_library(gnclib STATIC ${gnc_sources})
target_include_directories(gnclib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
	${extlibs}/eigen
)

set(interface_sources
${CMAKE_CURRENT_SOURCE_DIR}/network.cpp
${CMAKE_CURRENT_SOURCE_DIR}/MotorDriver.cpp
${CMAKE_CURRENT_SOURCE_DIR}/RCInput.cpp
${CMAKE_CURRENT_SOURCE_DIR}/IMUHandler.cpp
)
add_library(drone_interface STATIC ${interface_sources})
target_include_directories(drone_interface PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${extlibs}/eigen
)
target_link_libraries(drone_interface PUBLIC Common Navio2)
