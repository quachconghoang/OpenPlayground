#ifndef PCL_STORAGE_H
#define PCL_STORAGE_H

#include "pcl/common/common_headers.h"

#ifndef HOANGQC_POINTXYZ
#define HOANGQC_POINTXYZ
typedef pcl::PointXYZ							PointT;
typedef pcl::PointCloud<pcl::PointXYZ>			PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr		PointCloudPtrT;

typedef pcl::Normal								NormalT;
typedef pcl::PointCloud<pcl::Normal>			NormalCloudT;
typedef pcl::PointCloud<pcl::Normal>::Ptr		NormalCloudPtrT;
#endif

enum PLANE_VISUALIZATION_MODE{ PLANE_RAW, PLANE_GRID, PLANE_MESH, PLANE_NONE };

struct PlaneStorage
{
	pcl::ModelCoefficients::Ptr modelCoefficients;
	std::string tagID;

	PointCloudPtrT pointCloud;
	PointCloudPtrT hullCloud;
	std::vector<PointCloudT> blockCloud;

	PointCloudPtrT gridCloud;
	float gridCloud_CellSize;

	pcl::PolygonMesh::Ptr mesh;

	//RGBColor color;
	PointCloudPtrT capturePoints;
	PLANE_VISUALIZATION_MODE displayingMode;
	bool isHighlight;
};

class PCLStorage
{
	
public:
	PCLStorage();
	~PCLStorage();
	
	PointCloudPtrT cloud_input;
	std::string cloud_input_id;
	void setInputCloud(std::string fileName);

};

#endif