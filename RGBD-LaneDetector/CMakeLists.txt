cmake_minimum_required(VERSION 2.8)
project( LaneDetector )

set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
INCLUDE(CMakeModules/CudaUtils.cmake)
enable_cuda_support()

find_package( OpenCV REQUIRED )
find_package( Eigen REQUIRED )

#define include
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

file(GLOB ImgProcCUDA_src ./ImgProc/*.h ./ImgProc/*.cpp ./ImgProc/cuda/*.h ./ImgProc/cuda/*.cu)
file(GLOB Utils_src ./Utils/*.hpp ./Utils/*.cpp)

# cuda_add_library(libImgProcCUDA STATIC ${ImgProcCUDA_src} OPTIONS -std=c++11)
cuda_add_library(libImgProcCUDA STATIC ${ImgProcCUDA_src})
target_link_libraries( libImgProcCUDA ${OpenCV_LIBS})

#add_executable( Test_Pointcloud Test_Pointcloud.cpp )
#target_link_libraries( Test_Pointcloud ${OpenCV_LIBS} libImgProcCUDA)
#set_target_properties(Test_Pointcloud PROPERTIES LINK_FLAGS "/ignore:4244")

set(Build_RansacPlane false CACHE BOOL "Build pcl processing")
if(Build_RansacPlane)
	find_package( PCL REQUIRED )
	add_executable( Test_RansacPlane Test_RansacPCL.cpp ${Utils_src})
	target_include_directories( Test_RansacPlane PRIVATE ${PCL_INCLUDE_DIRS})
	target_link_libraries( Test_RansacPlane libImgProcCUDA ${OpenCV_LIBS} ${PCL_LIBRARIES} )
endif()

set(Build_Recorder false CACHE BOOL "Build realsense recorder")
if(Build_Recorder)
	find_package( RealSense REQUIRED)
	include_directories(${RealSense_INCLUDE_DIR})
	add_executable( RGBD_Recorder RGBD_Recorder.cpp )
	target_link_libraries( RGBD_Recorder ${OpenCV_LIBS} ${RealSense_LIBRARY})
endif()

set(Build_Tests true CACHE BOOL "Build pcl processing")
if(Build_Tests)
	# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
	add_executable( Test_TemplateMatching Test_TemplateMatching.cpp)
	target_link_libraries( Test_TemplateMatching ${OpenCV_LIBS} libImgProcCUDA)
	add_executable( Test_Matching_Synth Test_Matching_Synth.cpp)
	target_link_libraries( Test_Matching_Synth ${OpenCV_LIBS} libImgProcCUDA)
	
	add_executable( Test_Matching_ATC2017 Test_Matching_ATC2017.cpp)
	target_link_libraries( Test_Matching_ATC2017 ${OpenCV_LIBS} libImgProcCUDA)
	# set_target_properties(Test_Matching_ATC2017 PROPERTIES LINK_FLAGS "/ignore:4244")
endif()

