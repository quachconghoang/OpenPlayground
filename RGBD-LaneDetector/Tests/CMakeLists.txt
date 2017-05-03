cmake_minimum_required(VERSION 2.8)

if(Build_Tests)
	include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
	add_executable( Test_TemplateMatching Test_TemplateMatching.cpp)
	target_link_libraries( Test_TemplateMatching ${OpenCV_LIBS} libImgProcCUDA)
endif()
