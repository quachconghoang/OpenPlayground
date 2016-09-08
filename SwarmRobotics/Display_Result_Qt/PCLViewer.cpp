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

	pclViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	pclViewer->addCoordinateSystem(axeSize);
	qvtkWidget->SetRenderWindow(pclViewer->getRenderWindow());
	pclViewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
	qvtkWidget->update();

	//pclViewer->registerPointPickingCallback(boost::bind(&PCLViewer::pp_callback, this, _1, (void*)&pclViewer));
	pclViewer->registerMouseCallback(boost::bind(&PCLViewer::mouseEventOccurred, this, _1, (void*)&pclViewer));
}

void PCLViewer::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void)
{
	if (event.getButton() == pcl::visualization::MouseEvent::MouseScrollDown ||
		event.getButton() == pcl::visualization::MouseEvent::MouseScrollUp)
	{
		//qDebug() << "C++ Style Debug Message";
	}
}