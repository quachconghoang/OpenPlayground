#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

#include "pcl/visualization/pcl_visualizer.h"
#include "PCLStorage.h"
#include "QVTKWidget.h"

class PCLViewer
{
public:
	PCLViewer();
	~PCLViewer();

	void setupPCLViewer(QVTKWidget * _qvtkWidget, float axeSize = 1000);
	void displayRawData(PCLStorage & _cloudStorage);
	void displaySurfaces(PCLStorage * obj);

	pcl::visualization::PCLVisualizer::Ptr pclVisualizer;
	//PCLStorage cloudStorage;
	QVTKWidget * qvtkWidget;

private:
	void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);
	void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);
};

#endif // !PCL_VIEWER_H