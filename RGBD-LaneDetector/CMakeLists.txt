cmake_minimum_required(VERSION 2.8)
project( LaneDetector )

set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
INCLUDE(CMakeModules/CudaUtils.cmake)
enable_cuda_support()

find_package( OpenCV REQUIRED )
find_package( Eigen REQUIRED )
#find_package( PCL REQUIRED )

#define include
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

file(GLOB ImgProcCUDA_src ./ImgProc/*.h ./ImgProc/*.cpp ./ImgProc/cuda/*.h ./ImgProc/cuda/*.cu)

# cuda_add_library(libImgProcCUDA STATIC ${ImgProcCUDA_src} OPTIONS -std=c++11)
cuda_add_library(libImgProcCUDA STATIC ${ImgProcCUDA_src})
target_link_libraries( libImgProcCUDA ${OpenCV_LIBS})

add_executable( Test_Pointcloud Test_Pointcloud.cpp )
target_link_libraries( Test_Pointcloud ${OpenCV_LIBS} )

find_package( RealSense REQUIRED)
include_directories(${RealSense_INCLUDE_DIR})
add_executable( RGBD_Recorder RGBD_Recorder.cpp )
target_link_libraries( RGBD_Recorder ${OpenCV_LIBS} ${RealSense_LIBRARY})