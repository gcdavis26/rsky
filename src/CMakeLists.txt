project(ekf)
set(gnc_sources 
${CMAKE_CURRENT_SOURCE_DIR}/math_utils.cpp
${CMAKE_CURRENT_SOURCE_DIR}/ekf.cpp
${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp
)
add_library(gnclib STATIC ${gnc_sources})
target_include_directories(gnclib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
	${extlibs}/eigen
)
