#include "stdafx.h"
#include "PCLViewer.h"
#include "vtkRenderWindow.h"

#define kSurfacePrefix ".cloud_"
#define kHullPrefix ".hull_"
#define kMeshPrefix ".mesh_"
#define kCapturePointPrefix ".capturePoints_"

using namespace pcl::visualization;

PCLViewer::PCLViewer()
{
	
}


PCLViewer::~PCLViewer()
{

}


void PCLViewer::setupPCLViewer(QVTKWidget * _qvtkWidget, float axeSize /* = 1000 */)
{
	qvtkWidget = _qvtkWidget;

	pclVisualizer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	pclVisualizer->addCoordinateSystem(axeSize);
	qvtkWidget->SetRenderWindow(pclVisualizer->getRenderWindow());
	pclVisualizer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
	qvtkWidget->update();

	pclVisualizer->registerPointPickingCallback(boost::bind(&PCLViewer::pp_callback, this, _1, (void*)&pclVisualizer));

	startMesh = (new pcl::PolygonMesh);
	stopMesh = (new pcl::PolygonMesh);

	pcl::io::loadPLYFile("..\\..\\IndoorData\\flagStart.ply", *startMesh);
	pcl::io::loadPLYFile("..\\..\\IndoorData\\flagStop.ply", *stopMesh);
}

void PCLViewer::displayRawData(PCLStorage & _cloudStorage)
{
	if (_cloudStorage.cloud_input)
	{
		pclVisualizer->removeAllPointClouds();
		pclVisualizer->addPointCloud(_cloudStorage.cloud_input, _cloudStorage.tagID);
	}
}

void PCLViewer::displaySurfaces(PCLStorage * obj)
{
	for (int i = 0; i < obj->planes.size(); i++)
	{
		std::string tagID = obj->planes[i].tagID + kSurfacePrefix;
		std::string tagHull_ID = obj->planes[i].tagID + kHullPrefix;

		pclVisualizer->removeShape(tagHull_ID);
		pclVisualizer->removePointCloud(tagID);

		pclVisualizer->addPointCloud<PointT>(obj->planes[i].pointCloud, tagID);
		RGBColor cl = obj->planes[i].color;
		pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, cl.r, cl.g, cl.b, tagID);

		pclVisualizer->addPolygon<PointT>(obj->planes[i].hullCloud, tagHull_ID);
		pclVisualizer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, cl.r, cl.g, cl.b, tagHull_ID);

		obj->planes[i].displayingMode = PLANE_RAW;
	}
}

void PCLViewer::displayGridSurfaces(PCLStorage * obj)
{
	for (int i = 0; i < obj->planes.size(); i++)
	{
		std::string tagHull_ID = obj->planes[i].tagID + kHullPrefix;
		pclVisualizer->removeShape(tagHull_ID);
		pclVisualizer->addPolygon<PointT>(obj->planes[i].hullCloud, tagHull_ID);

		std::string tagGrid_ID = obj->planes[i].tagID + kSurfacePrefix;
		pclVisualizer->updatePointCloud<PointT>(obj->planes[i].gridCloud, tagGrid_ID);

		RGBColor cl = obj->planes[i].color;
		pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, cl.r, cl.g, cl.b, tagGrid_ID);
		//setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE,6,tagGrid_ID);
		pclVisualizer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, cl.r, cl.g, cl.b, tagHull_ID);
		obj->planes[i].displayingMode = PLANE_GRID;
	}
}

void PCLViewer::displayMeshSurfaces(PCLStorage * obj)
{
	for (int i = 0; i < obj->planes.size(); i++)
	{
		std::string tagMesh_ID = obj->planes[i].tagID + kMeshPrefix;
		pclVisualizer->addPolylineFromPolygonMesh(*obj->planes[i].mesh, tagMesh_ID);
		RGBColor cl = obj->planes[i].color;
		pclVisualizer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, cl.r, cl.g, cl.b, tagMesh_ID);
		pclVisualizer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, 1, tagMesh_ID);
		obj->planes[i].displayingMode = PLANE_MESH;
	}
}

void PCLViewer::hideMeshSurfaces(PCLStorage * obj)
{
	for (int i = 0; i < obj->planes.size(); i++)
	{
		std::string tagMesh_ID = obj->planes[i].tagID + kMeshPrefix;
		pclVisualizer->removeShape(tagMesh_ID);
		obj->planes[i].displayingMode = PLANE_GRID;
	}
}

void PCLViewer::highlightSurface(PCLStorage * obj, pcl::PointXYZ p)
{
	int minIndex = 0;
	float minValue = 30000.f;
	for (int i = 0; i < obj->planes.size(); i++){
		pcl::ModelCoefficients::Ptr model = obj->planes[i].modelCoefficients;
		float a = model->values[0];
		float b = model->values[1];
		float c = model->values[2];
		float d = model->values[3];
		float difVal = fabs(a*p.x + b*p.y + c*p.z + d);

		if (difVal < minValue){
			minIndex = i;
			minValue = difVal;
		}
	}
	if (minIndex != obj->selected_Index && minIndex < obj->planes.size()){
		if (obj->selected_Index != -1)
		{
			//REMOVE PREVIOUS HIGHLIGHT
			unHighlightSurfaces(obj);
		}
		std::string enable_ID = obj->planes[minIndex].tagID + kSurfacePrefix;
		setPointCloudSelected(true, enable_ID);
		PointCloudPtrT hull = obj->planes[minIndex].hullCloud;
		obj->selected_Index = minIndex;
		obj->visualConnector->planeDidSelected(true);
	}
	else
	{
		unHighlightSurfaces(obj);
	}
}

void  PCLViewer::unHighlightSurfaces(PCLStorage * obj)
{
	if (obj->selected_Index != -1){
		std::string m_ID = obj->planes[obj->selected_Index].tagID;
		setPointCloudSelected(false, m_ID + kSurfacePrefix);
		obj->selected_Index = -1;
		obj->visualConnector->planeDidSelected(false);
	}
}

bool PCLViewer::setPointCloudSelected(const bool selected, const std::string &id)
{
	CloudActorMap::iterator am_it = pclVisualizer->getCloudActorMap()->find(id);

	if (am_it == pclVisualizer->getCloudActorMap()->end()){
		pcl::console::print_error("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str());
		return (false);
	}
	// Get the actor pointer
	vtkLODActor* actor = vtkLODActor::SafeDownCast(am_it->second.actor);
	float psize = actor->GetProperty()->GetPointSize();
	if (selected){
		actor->GetProperty()->SetPointSize(psize + 5);
		actor->Modified();
	}
	else{
		if (psize - 5 < 1)psize = 1; else psize -= 5;
		actor->GetProperty()->SetPointSize(psize);
		actor->Modified();
	}
	return (true);
}

void transformMesh(pcl::PolygonMesh & inMesh, Eigen::Matrix4f transform, pcl::PolygonMesh & outMesh)
{
	//Important part starts here 
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::fromPCLPointCloud2(inMesh.cloud, cloud);
	pcl::transformPointCloud(cloud, cloud, transform);
	pcl::toPCLPointCloud2(cloud, outMesh.cloud);
	outMesh.polygons = inMesh.polygons;
}

void PCLViewer::displayCapturePoints(PCLStorage * obj, int pIndex)
{
	if (obj->planes[pIndex].capturePoints->size() > 0)
	{
		std::string tagCapturePoints = obj->planes[pIndex].tagID + kCapturePointPrefix;

		pcl::PolygonMesh wayMesh;
		pcl::toPCLPointCloud2(*obj->planes[pIndex].capturePoints, wayMesh.cloud);
		pcl::Vertices ver;
		for (int i = 0; i < obj->planes[pIndex].capturePoints->size(); i++){
			ver.vertices.push_back(i);
		}
		wayMesh.polygons.push_back(ver);
		pclVisualizer->addPolylineFromPolygonMesh(wayMesh, tagCapturePoints + "_line");

		//ADD NEW WAYPOINTS
		RGBColor tColor = obj->planes[pIndex].color;
		pclVisualizer->removePointCloud(tagCapturePoints);
		pclVisualizer->addPointCloud<PointT>(obj->planes[pIndex].capturePoints, tagCapturePoints);
		pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 5, tagCapturePoints);
		pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, tagCapturePoints);

		PointT startPoint = obj->planes[pIndex].capturePoints->points.front();
		PointT stopPoint = obj->planes[pIndex].capturePoints->points.back();
		Eigen::Affine3f startPose(Eigen::Translation3f(startPoint.x, startPoint.y, startPoint.z));
		Eigen::Affine3f stopPose(Eigen::Translation3f(stopPoint.x, stopPoint.y, stopPoint.z));

		pcl::PolygonMesh startFlag, stopFlag;
		transformMesh(*startMesh, startPose.matrix(),startFlag);
		transformMesh(*stopMesh, stopPose.matrix(), stopFlag);
		pclVisualizer->addPolygonMesh(startFlag, tagCapturePoints + "start");
		pclVisualizer->addPolygonMesh(stopFlag, tagCapturePoints + "stop");
	}
}

void PCLViewer::hideCapturePoints(PCLStorage * obj, int pIndex)
{
	if (obj->planes[pIndex].capturePoints)
	{
		std::string tagCapturePoints = obj->planes[pIndex].tagID + kCapturePointPrefix;
		//REMOVE OLD WAYPOINTS
		pclVisualizer->removePointCloud(tagCapturePoints);
		pclVisualizer->removeShape(tagCapturePoints + "_line");
		pclVisualizer->removeShape(tagCapturePoints + "start");
		pclVisualizer->removeShape(tagCapturePoints + "stop");
	}
}

void PCLViewer::pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	pclVisualizer->removeShape("sphere");
	pclVisualizer->addSphere(current_point, 100, 1, 0, 1, "sphere", 0);

	if (pclStorage->isSegmented)	highlightSurface(pclStorage, current_point);
}

void pushCubeToPolygon(PointCloudPtrT cloud, std::vector<pcl::Vertices>&vertices, PointT center, float size)
{
	using namespace std;
	int offset = cloud->points.size();

	float x0 = center.x - size / 2;
	float x1 = center.x + size / 2;
	float y0 = center.y - size / 2;
	float y1 = center.y + size / 2;

	float z0 = center.z - size / 2;
	float z1 = center.z + size / 2;

	//point 0-3
	cloud->push_back(PointT(x0, y0, z0));cloud->push_back(PointT(x1, y0, z0));cloud->push_back(PointT(x1, y1, z0));cloud->push_back(PointT(x0, y1, z0));

	//point 4-7
	cloud->push_back(PointT(x0, y0, z1));cloud->push_back(PointT(x1, y0, z1));cloud->push_back(PointT(x1, y1, z1));cloud->push_back(PointT(x0, y1, z1));

	pcl::Vertices face0, face1, face2, face3, face4, face5;
	int data0[] = { offset + 0, offset + 1, offset + 2, offset + 2, offset + 3, offset + 0 };
	copy(&data0[0], &data0[6], back_inserter(face0.vertices));

	int data1[] = { offset + 3, offset + 2, offset + 6, offset + 6, offset + 7, offset + 3 };
	copy(&data1[0], &data1[6], back_inserter(face1.vertices));

	int data2[] = { offset + 7, offset + 6, offset + 5, offset + 5, offset + 4, offset + 7 };
	copy(&data2[0], &data2[6], back_inserter(face2.vertices));

	int data3[] = { offset + 4, offset + 5, offset + 1, offset + 1, offset + 0, offset + 4 };
	copy(&data3[0], &data3[6], back_inserter(face3.vertices));

	int data4[] = { offset + 4, offset + 0, offset + 3, offset + 3, offset + 7, offset + 4 };
	copy(&data4[0], &data4[6], back_inserter(face4.vertices));

	int data5[] = { offset + 1, offset + 5, offset + 6, offset + 6, offset + 2, offset + 1 };
	copy(&data5[0], &data5[6], back_inserter(face5.vertices));

	vertices.push_back(face0);vertices.push_back(face1);vertices.push_back(face2);
	vertices.push_back(face3);vertices.push_back(face4);vertices.push_back(face5);
}

void PCLViewer::displayRemainCloudAsCubes(PCLStorage * obj, double resolution /* = 500 */)
{
	PointCloudPtrT cloud = obj->cloud_remain;
	pcl::octree::OctreePointCloudVoxelCentroid<PointT> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	std::vector<PointT, Eigen::aligned_allocator<PointT> > voxelCenters;
	octree.getOccupiedVoxelCenters(voxelCenters);
	double voxelSideLen = sqrt(octree.getVoxelSquaredSideLen());
	octree.deleteTree();

	PointCloudPtrT cube_cloud;
	cube_cloud.reset(new PointCloudT);
	std::vector<pcl::Vertices> cube_vertices;

	for (size_t i = 0; i < voxelCenters.size(); i++) {
		pushCubeToPolygon(cube_cloud, cube_vertices, voxelCenters[i], voxelSideLen);
	}
	pclVisualizer->addPolygonMesh<PointT>(cube_cloud, cube_vertices, obj->tagID + "." + "remain");
}

void PCLViewer::hideRemainCloudAsCubes(PCLStorage * obj)
{
	pclVisualizer->removeShape(obj->tagID + "." + "remain");
}