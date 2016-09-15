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