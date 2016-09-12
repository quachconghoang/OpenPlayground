#ifndef PLANAR_CREATEOR_H
#define PLANAR_CREATEOR_H

#include "stdafx.h"
#include <QProgressDialog.h>

#ifndef HOANGQC_POINTXYZ
#define HOANGQC_POINTXYZ
typedef pcl::PointXYZ							PointT;
typedef pcl::PointCloud<pcl::PointXYZ>			PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr		PointCloudPtrT;

typedef pcl::Normal								NormalT;
typedef pcl::PointCloud<pcl::Normal>			NormalCloudT;
typedef pcl::PointCloud<pcl::Normal>::Ptr		NormalCloudPtrT;
#endif

enum TYPE_PROJECTED_PLANE
{
	TYPE_PROJECTED_PLANE_001,
	TYPE_PROJECTED_PLANE_010,
	TYPE_PROJECTED_PLANE_100
};


namespace helper
{
	using namespace pcl;
	using namespace pcl::visualization;
	
	PointCloudPtrT downSampling(PointCloudPtrT & targetCloud,float leafSize);

	NormalCloudPtrT normalsEstimate(PointCloudPtrT cloud_input, int neighbors);

	double areaEstimate(PointCloudPtrT cloud_input, double resolution);

	PointCloudPtrT statisticalOutlierRemoval(PointCloudPtrT cloud_input, int neighbors);

	PointCloudPtrT projectCloud(PointCloudPtrT cloud_input,pcl::ModelCoefficients::Ptr coefficients);

	PointCloudPtrT hullEsitmate(PointCloudPtrT cloud_input);

	pcl::PolygonMesh::Ptr triangulation(PointCloudPtrT cloud_input);
	
	PointCloudPtrT generateDisplayCloud(const pcl::ModelCoefficients::Ptr coefficients, const PointCloudPtrT boundary, const std::vector<PointCloudT> & blockBoundary);

	PointCloudPtrT generate_CapturePoint(const pcl::ModelCoefficients::Ptr coefficients, const PointCloudPtrT boundary, std::vector<PointCloudT> & blockBoundary,float capture_width, float capture_height, float attitude);

	void generateBlockingBoundary(PointCloudPtrT planeCloud, PointCloudPtrT hullCloud, pcl::ModelCoefficients::Ptr planeModel, std::vector<PointCloudT> & blockBoundary);
}

pcl::ModelCoefficients::Ptr estimateBestNorm(Eigen::Vector3f norm_model, Eigen::Vector3i & point_index);

#endif




