cmake_minimum_required(VERSION 3.10.0)
project(franka VERSION 0.1.0 LANGUAGES C CXX)

set(EIGEN3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/eigen)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(franka CIC_controller.cpp PID_controller.cpp)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
