// Test_Visualization.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <conio.h>
#include <vector>
#include <iostream>

#include "opencv2/opencv.hpp"
#include "pcl/filters/voxel_grid.h"

using namespace pcl;
using namespace pcl::visualization;

bool MODE_SEGMENTED = false;

PCLVisualizer::Ptr viewer;

void printUsage(const char* progName);
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;
	}

	if (event.getKeySym() == "F1" && event.keyDown() && !MODE_SEGMENTED )
	{
		PCL_WARN("Segmentation ... \n");
		MODE_SEGMENTED = true;
	}
}

void pp_callback(const pcl::visualization::PointPickingEvent& event)
{
	if (event.getPointIndex() == -1)
		return;
	PointXYZ current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int _tmain(int argc, _TCHAR* argv[])
{
	PointCloud<PointXYZ>::Ptr worldCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr wp(new PointCloud<PointXYZ>);

	pcl::io::loadPCDFile("..\\..\\IndoorData\\environment.pcd", *worldCloud);
	pcl::io::loadPCDFile("..\\..\\IndoorData\\CapturePoints_Large.pcd", *wp);


	viewer.reset(new PCLVisualizer);
	viewer->setSize(640, 480);
	viewer->addCoordinateSystem(1000.0);
	
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerPointPickingCallback(pp_callback);


	viewer->addPointCloud(worldCloud, "world");

	viewer->addPointCloud(wp, "points");
	viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1, 0, 0, "points");
	viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5, "points");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

void printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "\n\n";
}