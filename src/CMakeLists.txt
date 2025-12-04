project(ekf)
set(ekf_sources 
${CMAKE_CURRENT_SOURCE_DIR}/math_utils.cpp
${CMAKE_CURRENT_SOURCE_DIR}/ekf.cpp
)
add_library(ekflib STATIC ${ekf_sources})
target_include_directories(ekflib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
	${extlibs}/eigen
)
