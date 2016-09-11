#include "stdafx.h"
#include "PCLStorage.h"
#include "pcl/common/common_headers.h"


PCLStorage::PCLStorage()
{
	isSegmented = false;
}

PCLStorage::~PCLStorage()
{
}

void PCLStorage::setInputCloud(std::string fileName)
{
	cloud_input.reset(new PointCloudT);
	pcl::io::loadPCDFile(fileName, *cloud_input);
}

void PCLStorage::segmentPointcloud(double minPlaneArea, 
	double disThreshold, 
	double maxIter, 
	int maxRetry, 
	int minClusterSize, 
	double clusterThreshold)
{

	PointCloudPtrT cloud_blob(new PointCloudT);
	PointCloudPtrT cloud_remain_temp(new PointCloudT);
	PointCloudPtrT cloud_projected(new PointCloudT);
	pcl::copyPointCloud(*cloud_input, *cloud_blob);
	PointCloudPtrT cloud_f(new PointCloudT);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


	isSegmented = true;
}