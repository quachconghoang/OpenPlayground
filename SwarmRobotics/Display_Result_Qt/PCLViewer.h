#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

//#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "QVTKWidget.h"

class PCLViewer
{
public:
	PCLViewer();
	~PCLViewer();

	void setupPCLViewer(QVTKWidget * _qvtkWidget, float axeSize = 1000);

	pcl::visualization::PCLVisualizer::Ptr pclViewer;
	QVTKWidget * qvtkWidget;

private:
	void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);
};

#endif // !PCL_VIEWER_H