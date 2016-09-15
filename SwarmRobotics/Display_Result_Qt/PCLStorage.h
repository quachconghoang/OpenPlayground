#ifndef PCL_STORAGE_H
#define PCL_STORAGE_H

#include "pcl/common/common_headers.h"
#include "PlanarCreator.h"
#include <boost/container/vector.hpp>
#include <random>
#include "VisualConnector.h"

#ifndef HOANGQC_POINTXYZ
#define HOANGQC_POINTXYZ
typedef pcl::PointXYZ							PointT;
typedef pcl::PointCloud<pcl::PointXYZ>			PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr		PointCloudPtrT;

typedef pcl::Normal								NormalT;
typedef pcl::PointCloud<pcl::Normal>			NormalCloudT;
typedef pcl::PointCloud<pcl::Normal>::Ptr		NormalCloudPtrT;
#endif

struct RGBColor{	double r;	double g;	double b;	};
enum PLANE_VISUALIZATION_MODE{ PLANE_RAW, PLANE_GRID, PLANE_MESH, PLANE_NONE };

class PlaneStorage
{
public:
	PlaneStorage(std::string iD = "default.0");
	~PlaneStorage(void);

	pcl::ModelCoefficients::Ptr modelCoefficients;
	std::string tagID;

	PointCloudPtrT pointCloud;
	PointCloudPtrT hullCloud;
	std::vector<PointCloudT> blockCloud;

	PointCloudPtrT gridCloud;
	float gridCloud_CellSize;

	pcl::PolygonMesh::Ptr mesh;

	RGBColor color;
	PointCloudPtrT capturePoints;
	PLANE_VISUALIZATION_MODE displayingMode;
	bool isHighlight;

	float area;

	void clear();
};

class PCLStorage
{
	
public:
	PCLStorage();
	~PCLStorage();
	
	PointCloudPtrT cloud_input;
	PointCloudPtrT cloud_remain;
	std::string cloud_input_id;
	bool isSegmented;
	int selected_Index;
	VisualConnector * visualConnector;

	std::string tagID;
	boost::container::vector<PlaneStorage> planes;

	void setInputCloud(std::string fileName);
	void clearData();

	void segmentParams(std::vector<double> params);
	void segmentPointcloud(double minPlaneArea, 
		double disThreshold, 
		double maxIter, 
		int maxRetry, 
		int minClusterSize, 
		double clusterThreshold);
	int createCapturePoint(uint cloud_index, 
		float capture_width, 
		float capture_height, 
		float attitude);
	bool thePlaneHasWaypoint();
	int saveWaypoint(std::string fileName);

private:

	cv::RNG genRandom;
	//std::mt19937 gen;
	//void calculateBoundModel(PointCloudPtrT cloud_input);
};

#endif