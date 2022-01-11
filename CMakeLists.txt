#  @author dingyi
#  @start_date 01/10/2022
#  @modified_date 01/10/2022
#  @stereo_calibration

cmake_minimum_required(VERSION 3.20)        # CMake version check
project(stereo-camera-calibration)          # Create project
set(CMAKE_CXX_STANDARD 14)                  # Enable c++14 standard

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stereo-camera-calibration main.cpp calibrate_stereo.cpp calibrate_stereo.h)
target_link_libraries(stereo-camera-calibration ${OpenCV_LIBS})