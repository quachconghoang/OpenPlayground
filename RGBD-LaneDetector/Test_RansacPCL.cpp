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

std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-SUMMER/";
int count = 150;
#define RAW_IMG_WIDTH 1280
#define	RAW_IMG_HEIGHT 760

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
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);
	//viewer = UTIL::rgbVis("RANSAC - Raw", true);
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Synthia_RGBD_HALF);

	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		cv::resize(img, img, cv::Size(RAW_IMG_WIDTH / 2, RAW_IMG_HEIGHT/2));
		cv::resize(dimg, dimg, cv::Size(RAW_IMG_WIDTH / 2, RAW_IMG_HEIGHT / 2));


		pcl::cuda::PointCloudAOS<pcl::cuda::Host> data;
		pcl::cuda::PointCloudAOS<pcl::cuda::Device> dev_data;


		cv::imshow("RGB", img);
		cv::imshow("D", dimg);
		cv::waitKey();
		count++;
	}



	
	
	
}