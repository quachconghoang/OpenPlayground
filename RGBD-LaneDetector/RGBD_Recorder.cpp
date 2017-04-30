#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include "opencv2/opencv.hpp"


bool programShouldClose;
#define _UPDATE_TIME  0.2
std::string folderName = "Sample";
//std::string dirPath = "Sample/";
std::string depthDir = "depth/";
std::string colorDir = "rgb/";
std::ofstream recordFile;

void setupInputFolder();
int runRealsense();
void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);
//int runOpenNI2();

int main()
{
	runRealsense();
}

#include "librealsense/rs.hpp"
int runRealsense()
{
	cv::Mat depthMat, colorMat, depthVizMat;
	
	setupInputFolder();

	// Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
	rs::log_to_console(rs::log_severity::warn);
	// Create a context object. This object owns the handles to all connected realsense devices.
	rs::context ctx;

	printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
	if (ctx.get_device_count() == 0) return EXIT_FAILURE;

	// This tutorial will access only a single device, but it is trivial to extend to multiple devices
	rs::device * dev = ctx.get_device(0);
	printf("\nUsing device 0, an %s\n", dev->get_name());
	printf("    Serial number: %s\n", dev->get_serial());
	printf("    Firmware version: %s\n", dev->get_firmware_version());
	printf("	Press R to start Recording ... \n");

	// Configure depth and color to run with the device's preferred settings
	dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
	dev->set_option(rs::option::r200_lr_auto_exposure_enabled,true);
	dev->set_option(rs::option::color_enable_auto_exposure, true);
	dev->set_option(rs::option::color_enable_auto_white_balance, true);
	dev->start();

	// Retrieve camera parameters for mapping between depth and color
	rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
	rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
	rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
	
	recordFile << "# Depth instr: " << std::fixed
		<< depth_intrin.width << " "
		<< depth_intrin.height << " "
		<< depth_intrin.fx << " "
		<< depth_intrin.fy << " "
		<< depth_intrin.ppx << " "
		<< depth_intrin.ppy << "\n";
	depthVizMat = cv::Mat(depth_intrin.height, depth_intrin.width, CV_8UC3);

	int indexData = 0;
	bool isRecording = false;
	int frameCount = 0;
	int64 st = cv::getTickCount();

	while (!programShouldClose)
	{
		dev->wait_for_frames();
		frameCount++;
		int64 ct = cv::getTickCount();
		double _sec = (ct - st) / cv::getTickFrequency();
		if (_sec > _UPDATE_TIME){
			printf("FPS: %d \n",int(frameCount/_sec));
			st = cv::getTickCount();
			frameCount=0;
		}

		// Retrieve our images
		depthMat = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (ushort*)dev->get_frame_data(rs::stream::depth));
		colorMat = cv::Mat(depth_intrin.height, depth_intrin.width, CV_8UC3, (uchar*)dev->get_frame_data(rs::stream::color_aligned_to_depth));
		/*depthMat = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (ushort*)dev->get_frame_data(rs::stream::depth_aligned_to_color));
		colorMat = cv::Mat(depth_intrin.height, depth_intrin.width, CV_8UC3, (uchar*)dev->get_frame_data(rs::stream::color));*/

		cv::Mat bgrMat;
		if (frameCount == 0)
		{
			cv::cvtColor(colorMat, bgrMat, cv::COLOR_RGB2BGR);
			colorizeDepthImage(depthMat, depthVizMat);
			cv::imshow("depth IMG", depthVizMat);
			cv::imshow("color IMG", bgrMat);
		}
		
		/* === Record: === */
		if (isRecording && frameCount == 0)
		{
			std::string depFile = depthDir + std::to_string(indexData) + ".png";
			std::string rgbFile = colorDir + std::to_string(indexData) + ".png";
			recordFile << std::to_string(indexData) << " " << depFile << " " << std::to_string(indexData) << " " << rgbFile << "\n";

			
			cv::imwrite(folderName + "/" + depFile, depthMat);
			cv::imwrite(folderName + "/" + rgbFile, bgrMat);
			indexData++;
		}
		
		char key = cv::waitKey(5);
		if (key == 'q'){ programShouldClose = true; }
		if (key == 'r'){ isRecording = true; } // Press R to start Recording
	}
	
	recordFile.close();

	return 0;
}

void setupInputFolder()
{
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss; oss << std::put_time(&tm, "_%d_%m_%Y_%H_%M_%S");
	//folderName += oss.str();
	system((std::string("mkdir ") + folderName).c_str());
	system((std::string("cd ") + folderName + " && mkdir depth && mkdir rgb && cd ..").c_str());
	recordFile.open(folderName  + "/" + "associations.txt");

	recordFile << "# RGBD recorder TUM-format (Scale = 1x millimeter = 1000 ) \n";
	recordFile << "# Date: " << std::put_time(&tm, "%d-%m-%Y %H:%M:%S") << std::endl;
}

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat)
{
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			ushort realDepth = depthImg.at<ushort>(i, j)/5;
			int valCase = (realDepth - 50) / 256;
			uchar n = uchar((realDepth - 50) % 256);
			if (realDepth < 50) valCase = -1;

			switch (valCase){
			case 0:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, n, 255); break;
			case 1:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 255 - n); break;
			case 2:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(n, 255, 0); break;
			case 3:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255 - n, 0); break;
			case 4:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, n); break;
			case 5:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, n, 255); break;
			case 6:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255 - n); break;
			case 7:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255 - n, 0); break;
			case 8:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255 - n, 0, 0); break;
			default:	resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);	break;
			}
		}
	}
}