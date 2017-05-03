#pragma warning( disable : 4996 4800 )

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "pcl/pcl_base.h"

#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/sample_consensus/ransac.h>

#include "Utils/PCL_Utils.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

std::string folderName = "D:/LaneData/Kinect_DATA/";
pcl::visualization::PCLVisualizer::Ptr viewer;

void kinect2pointcloud()
{

}

void main()
{
	viewer = UTIL::rgbVis("RANSAC - Raw", true);
	//UTIL::
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::io::loadPLYFile<pcl::PointXYZRGB>(folderName + "005.ply", *cloud);

	//pcl::PointCloudAOS<Host> data_host;
	UTIL::displayColorPC(viewer, cloud);
	viewer->spin();
}