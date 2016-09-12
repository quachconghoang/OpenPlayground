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
	pclVisualizer->registerMouseCallback(boost::bind(&PCLViewer::mouseEventOccurred, this, _1, (void*)&pclVisualizer));
}

void PCLViewer::displayRawData(PCLStorage & _cloudStorage)
{
	if (_cloudStorage.cloud_input)
	{
		pclVisualizer->removeAllPointClouds();
		pclVisualizer->addPointCloud(_cloudStorage.cloud_input, _cloudStorage.cloud_input_id);
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
	}
}

void PCLViewer::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	if (event.getButton() == pcl::visualization::MouseEvent::MouseScrollDown ||
		event.getButton() == pcl::visualization::MouseEvent::MouseScrollUp)
	{
		//qDebug() << "C++ Style Debug Message";
	}
}

void PCLViewer::pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{

}