#include "stdafx.h"
#include "PCLViewer.h"
#include "vtkRenderWindow.h"


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

void PCLViewer::displayRawData()
{
	if (cloudStorage.cloud_input)
	{
		pclVisualizer->removeAllPointClouds();
		pclVisualizer->addPointCloud(cloudStorage.cloud_input, cloudStorage.cloud_input_id);
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