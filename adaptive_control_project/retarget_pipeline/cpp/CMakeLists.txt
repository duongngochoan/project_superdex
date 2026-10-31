cmake_minimum_required(VERSION 3.16)
project(retarget_controllers LANGUAGES CXX)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
file(GLOB CONTROLLER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(retarget_controllers SHARED ${CONTROLLER_SOURCES})
target_compile_features(retarget_controllers PRIVATE cxx_std_17)
target_link_libraries(retarget_controllers PRIVATE Eigen3::Eigen ${CMAKE_DL_LIBS})
set_target_properties(retarget_controllers PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../_native")
