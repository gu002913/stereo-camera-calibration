#  @author dingyi
#  @start_date 01/10/2022
#  @modified_date 05/31/2022
#  @stereo_calibration

cmake_minimum_required(VERSION 3.20)        # CMake version check
set(PROJECT_NAME "stereo-camera-calibration")
project(${PROJECT_NAME})          # Create project
set(CMAKE_CXX_STANDARD 14)                  # Enable c++14 standard

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


#--------------------------------------------------------------------------------
# SETTINGS
#--------------------------------------------------------------------------------

# Sources
set(SOURCES
        src/main.cpp
        src/calibrate_stereo.cpp

        )
set(HEADERS
        hpps/calibrate_stereo.h
        )


add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(stereo-camera-calibration ${OpenCV_LIBS})