#include "PCL_Utils.hpp"

int planeID = 0;
int cloudID = 0;

namespace UTIL
{
	using namespace pcl;
	using namespace pcl::visualization;

	boost::shared_ptr<PCLVisualizer> rgbVis(std::string name, bool axes)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));
		viewer->setBackgroundColor(0, 0, 0);

		if (axes) viewer->addCoordinateSystem(1, "World");
		viewer->initCameraParameters();
		viewer->setSize(640, 480);
		//viewer->setCameraPosition(2, 1, 1, 2.5, 2, 1, 0, 0, 1);
		viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, 1, 0);
		viewer->setCameraFieldOfView(pcl::deg2rad(46.6));
		//viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

		return (viewer);
	}

	void displayPC(PCLVisualizer::Ptr v, PointCloud<PointXYZ>::Ptr cloud, double r, double g, double b, double pSize)
	{
		std::string tTag = "CLOUD_" + std::to_string(cloudID++); //PCL_INFO("Display cloud %s \n", tTag.c_str());
		v->addPointCloud<PointXYZ>(cloud, tTag);
		v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, tTag);
		v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pSize, tTag);
	}

	void displayPC(PCLVisualizer::Ptr v, PointCloud<PointXYZRGBA>::Ptr cloud, double r, double g, double b, double pSize)
	{
		std::string tTag = "CLOUD_" + std::to_string(cloudID++); //PCL_INFO("Display cloud %s \n", tTag.c_str());
		v->addPointCloud<PointXYZRGBA>(cloud, tTag);
		v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, tTag);
		v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pSize, tTag);
	}

	void displayColorPC(PCLVisualizer::Ptr v, PointCloud<PointXYZRGB>::Ptr cloud, double pSize)
	{
		std::string tTag = "CLOUD_" + std::to_string(cloudID++);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		v->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, tTag);
		v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pSize, tTag);
	}

	void displayAxe(PCLVisualizer::Ptr v, Eigen::Vector3f og, Eigen::Vector3f v_x, Eigen::Vector3f v_y, Eigen::Vector3f v_z, std::string aTag)
	{
		float scale = 1.f;
		PointXYZ org = PointXYZ(og[0], og[1], og[2]);
		PointXYZ px = PointXYZ(org.x + v_x[0] * scale, org.y + v_x[1] * scale, org.z + v_x[2] * scale);
		PointXYZ py = PointXYZ(org.x + v_y[0] * scale, org.y + v_y[1] * scale, org.z + v_y[2] * scale);
		PointXYZ pz = PointXYZ(org.x + v_z[0] * scale, org.y + v_z[1] * scale, org.z + v_z[2] * scale);

		v->addArrow(px, org, 1, 0, 0, false, aTag + "_X");
		v->addArrow(py, org, 0, 1, 0, false, aTag + "_Y");
		v->addArrow(pz, org, 0, 0, 1, false, aTag + "_Z");
	}

	/*void connectPoints(PCLVisualizer::Ptr v, PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2, std::string lineID)
	{
		for (int i = 0; i < cloud1->points.size(); i++)
		{
			PointXYZ p1 = cloud1->points[i];
			PointXYZ p2 = cloud2->points[i];
			v->addLine(p1, p2, lineID + std::to_string(i));
		}
	}*/

	/*void displayCube(PCLVisualizer::Ptr v, PointXYZ p_min, Eigen::Vector3f size3, std::string cubeID)
	{
		PointXYZ p_max = PointXYZ(p_min.x + size3[0], p_min.y + size3[1], p_min.z + size3[2]);

		v->addCube(p_min.x, p_max.x, p_min.y, p_max.y, p_min.z, p_max.z, 1, 1, 1, cubeID);
		v->setShapeRenderingProperties(PCL_VISUALIZER_SHADING, PCL_VISUALIZER_SHADING_FLAT, cubeID);
		v->setShapeRenderingProperties(PCL_VISUALIZER_REPRESENTATION, PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeID);
	}*/

	/*void displayAxePCA(PCLVisualizer::Ptr v, Eigen::Matrix3f ev, Eigen::Vector4f mean, std::string axeID)
	{
		Eigen::Vector3f oP = Eigen::Vector3f(mean[0], mean[1], mean[2]);
		Eigen::Vector3f v_x = Eigen::Vector3f(ev(0, 0), ev(1, 0), ev(2, 0));
		Eigen::Vector3f v_y = Eigen::Vector3f(ev(0, 1), ev(1, 1), ev(2, 1));
		Eigen::Vector3f v_z = v_x.cross(v_y);

		float scale = 0.1f;
		PointXYZ org = PointXYZ(mean[0], mean[1], mean[2]);
		PointXYZ px = PointXYZ(org.x + v_x[0] * scale, org.y + v_x[1] * scale, org.z + v_x[2] * scale);
		PointXYZ py = PointXYZ(org.x + v_y[0] * scale, org.y + v_y[1] * scale, org.z + v_y[2] * scale);
		PointXYZ pz = PointXYZ(org.x + v_z[0] * scale, org.y + v_z[1] * scale, org.z + v_z[2] * scale);

		v->addArrow(px, org, 1, 0, 0, false, axeID + "_X");
		v->addArrow(py, org, 0, 1, 0, false, axeID + "_Y");
	}*/

	//void getMaskFromPC(cv::Mat & rMat, PointCloud<PointXYZ>::Ptr pc, cv::Vec3b colorCode)
	//{
	//	for (int i = 0; i < pc->height; i++){
	//		for (int j = 0; j < pc->width; j++){
	//			PointXYZ p = pc->at(j, i);//x-y
	//			if (pcl::isFinite(p)){ rMat.at<cv::Vec3b>(rMat.rows - i, j) = colorCode; }
	//		}
	//	}
	//}

}