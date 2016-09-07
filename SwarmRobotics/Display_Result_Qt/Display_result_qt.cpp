#include "Display_result_qt.h"
#include "vtkRenderWindow.h"

Display_Result_Qt::Display_Result_Qt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// Set up the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addCoordinateSystem(1000);
	vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow();
	ui.qvtkWidget->SetRenderWindow(renderWindow);
	ui.qvtkWidget->update();
}

Display_Result_Qt::~Display_Result_Qt()
{

}
