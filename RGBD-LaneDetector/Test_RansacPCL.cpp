#pragma warning( disable : 4996 4800 )

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "pcl/pcl_base.h"

#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/sample_consensus/ransac.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

#include "Utils/PCL_Utils.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/LaneDetector.h"
#include "ImgProc/DataIO.h"

std::string dirPath = "D:/LaneData/Sample_30-04/";
int count = 2100;
#define IMGWIDTH 640
#define	IMGHEIGHT 480

pcl::visualization::PCLVisualizer::Ptr viewer;
cv::Vec4f planeModel;

cv::Vec4f processModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setLeafSize(0.5, 0.5, 0.5);
	sor.setInputCloud(cloud);
	sor.filter(*filtered_Cloud);

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(filtered_Cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	ransac.setDistanceThreshold(.3);
	//ransac.setProbability(0);
	ransac.setMaxIterations(300);

	ransac.computeModel();

	Eigen::VectorXf model_coefficients;
	ransac.getModelCoefficients(model_coefficients);
	std::cout << "\n MODEL: \n" <<model_coefficients;

	UTIL::displayPC(viewer, filtered_Cloud, 0, 1, 0, 3);
	viewer->spinOnce();
	return cv::Vec4f(model_coefficients[0], model_coefficients[1], model_coefficients[2], model_coefficients[3]);
}

void main()
{
	ImgProc3D::LaneDetector laneDetector(640,480);
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);
	viewer = UTIL::rgbVis("RANSAC - Raw", true);

	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		laneDetector.processFrame(img, dimg);
		

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
		for (int i = 0; i < laneDetector.samplePoints.size();i++)
		{
			cloud->push_back(pcl::PointXYZ(laneDetector.samplePoints[i].x, laneDetector.samplePoints[i].y, laneDetector.samplePoints[i].z));
		}
		//std::cout << *cloud;
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		UTIL::displayPC(viewer,cloud,1,0,0);
		viewer->spinOnce();

		planeModel = processModel(cloud);
		laneDetector.genarateMap2D(planeModel);

		cv::putText(img, std::to_string(count), cv::Point(500, 420), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		cv::imshow("RGB", img);
		cv::imshow("D", dimg);
		cv::waitKey();
		count++;
	}



	
	
	
}