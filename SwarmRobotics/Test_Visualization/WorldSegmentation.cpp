#include "stdafx.h"
#include "WorldSegmentation.h"

WorldSegmentation::WorldSegmentation()
{
}


WorldSegmentation::~WorldSegmentation()
{
}

PointCloudPtrT helper::projectCloud(PointCloudPtrT cloud_input, pcl::ModelCoefficients::Ptr coefficients)
{
	PointCloudPtrT cloud_projected(new PointCloudT);
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_input);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	return cloud_projected;
}

PointCloudPtrT helper::statisticalOutlierRemoval(PointCloudPtrT cloud_input, int neighbors)
{
	PointCloudPtrT result_cloud(new PointCloudT);
	std::cerr << "Outline Filtering..." << std::endl;
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_input);
	sor.setMeanK(neighbors);
	sor.setStddevMulThresh(1);
	sor.filter(*result_cloud);
	return result_cloud;
}