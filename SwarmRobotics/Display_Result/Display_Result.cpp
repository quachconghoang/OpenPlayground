// Display_Result.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "pcl/common/transforms.h"


using namespace pcl;
using namespace pcl::visualization;

PCLVisualizer::Ptr viewer;
PolygonMesh startFlag;
PolygonMesh stopFlag;

PointCloud<PointXYZRGBA>::Ptr convert_VisCapturePoints(PointCloud<PointXYZ>::Ptr waypoints)
{
	PointCloud<PointXYZRGBA>::Ptr visPoints(new PointCloud<PointXYZRGBA>);
	size_t _sw = waypoints->size();
	//size_t _seg = _sw / 3;
	for (int i = 0; i < waypoints->size(); i++)
	{
		PointXYZ p = waypoints->points[i];
		PointXYZRGBA pc;
		pc.x = p.x; pc.y = p.y; pc.z = p.z;
		pc.r = 255 * ( double(i) / _sw );
		pc.g = 255 * ( 1 - double(i)/_sw );
		pc.b = 0;

		visPoints->push_back(pc);
	}
	return visPoints;
}

void transformMesh(pcl::PolygonMesh & inMesh, Eigen::Matrix4f transform)
{
	//Important part starts here 
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::fromPCLPointCloud2(inMesh.cloud, cloud);
	pcl::transformPointCloud(cloud, cloud, transform);
	pcl::toPCLPointCloud2(cloud, inMesh.cloud);
}

void drawMesh(PCLVisualizer::Ptr _viewer, PointCloud<PointXYZ>::Ptr waypoints)
{
	PointCloud<PointXYZRGBA>::Ptr visPoints = convert_VisCapturePoints(waypoints);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(visPoints);
	viewer->addPointCloud<pcl::PointXYZRGBA>(visPoints, rgb, "wp-cloud");
	viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 8, "wp-cloud");

	pcl::PolygonMesh wayMesh;
	pcl::toPCLPointCloud2(*visPoints, wayMesh.cloud);
	pcl::Vertices ver;
	for (int i = 0; i < visPoints->size(); i++)
	{ 
		ver.vertices.push_back(i); 
	}
	wayMesh.polygons.push_back(ver);

	_viewer->addPolylineFromPolygonMesh(wayMesh);
	PointXYZ startPoint = waypoints->points.front();
	PointXYZ stopPoint = waypoints->points.back();

	Eigen::Affine3f startPose(Eigen::Translation3f(startPoint.x, startPoint.y, startPoint.z));
	Eigen::Affine3f stopPose(Eigen::Translation3f(stopPoint.x, stopPoint.y, stopPoint.z));
	transformMesh(startFlag, startPose.matrix());
	transformMesh(stopFlag, stopPose.matrix());
	_viewer->addPolygonMesh(startFlag, "start");
	_viewer->addPolygonMesh(stopFlag, "stop");
}

int _tmain(int argc, _TCHAR* argv[])
{
	PointCloud<PointXYZ>::Ptr worldCloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr wp(new PointCloud<PointXYZ>);

	pcl::io::loadPCDFile("..\\..\\IndoorData\\environment.pcd", *worldCloud);
	pcl::io::loadPCDFile("..\\..\\IndoorData\\capturepoint_0.pcd", *wp);

	pcl::io::loadPLYFile("..\\..\\IndoorData\\flagStart.ply",startFlag);
	pcl::io::loadPLYFile("..\\..\\IndoorData\\flagStop.ply", stopFlag);

	viewer.reset(new PCLVisualizer);
	viewer->setSize(640, 480);
	viewer->addCoordinateSystem(1000.0);

	drawMesh(viewer, wp);

	viewer->addPointCloud(worldCloud, "world");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

