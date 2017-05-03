#include "opencv2/opencv.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Eigen/dense"
#include <sys/stat.h>
#include <pcl/console/parse.h>

#define ColorRAND (double(rand()%200+55))/255

namespace UTIL
{
	using namespace pcl;
	using namespace pcl::visualization;

	boost::shared_ptr<PCLVisualizer> rgbVis(std::string name, bool axes = true);
	void displayPC(PCLVisualizer::Ptr v, PointCloud<PointXYZ>::Ptr cloud, double r, double g, double b, double pSize = 1);
	void displayPC(PCLVisualizer::Ptr v, PointCloud<PointXYZRGBA>::Ptr cloud, double r, double g, double b, double pSize = 1);
	void displayColorPC(PCLVisualizer::Ptr v, PointCloud<PointXYZRGB>::Ptr cloud, double pSize = 1);

#ifdef _WIN32
	inline bool exists_test_fastest(const std::string& name) {
		struct stat buffer;
		return (stat(name.c_str(), &buffer) == 0);
	}

	inline double DisplayTime(std::string msg,int64 startTime)
	{
		double tTime = (cv::getTickCount() - startTime) / cv::getTickFrequency();
		PCL_INFO(" - %s : %f \n", msg.c_str(), tTime);
		return tTime;
	}
#endif

	void displayAxe(PCLVisualizer::Ptr v, Eigen::Vector3f og, Eigen::Vector3f v_x, Eigen::Vector3f v_y, Eigen::Vector3f v_z, std::string aTag);
	//void connectPoints(PCLVisualizer::Ptr v, PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2, std::string lineID);
	//void displayCube(PCLVisualizer::Ptr v, PointXYZ p_min, Eigen::Vector3f size3, std::string cubeID);
	//void displayAxePCA(PCLVisualizer::Ptr v, Eigen::Matrix3f ev, Eigen::Vector4f mean, std::string axeID);
	//void getMaskFromPC(cv::Mat & rMat, PointCloud<PointXYZ>::Ptr pc, cv::Vec3b colorCode);

}