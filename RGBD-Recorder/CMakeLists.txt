cmake_minimum_required(VERSION 2.8)
project( RGBD-Recorder )

set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )
find_package( RealSense REQUIRED)
find_package( OpenNI2 REQUIRED)

#define include
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${RealSense_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIRS})

add_executable( TumRecorder 
TumRecorder.cpp )
target_link_libraries( TumRecorder ${OpenCV_LIBS} )
target_link_libraries( TumRecorder ${RealSense_LIBRARY} )
target_link_libraries( TumRecorder ${OPENNI2_LIBRARY} )