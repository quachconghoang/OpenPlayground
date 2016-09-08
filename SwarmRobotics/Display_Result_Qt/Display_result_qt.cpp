#include "stdafx.h"
#include "Display_Result_Qt.h"

Display_Result_Qt::Display_Result_Qt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	viewer3D.setupPCLViewer(ui.qvtkWidget);
	//m_viewer->registerPointPickingCallback(boost::bind(&UAVoperator::pp_callback, this, _1, (void*)&m_viewer));
	//m_viewer->registerMouseCallback(boost::bind(&UAVoperator::mouseEventOccurred, this, _1, (void*)&m_viewer));

	connect(ui.actionOpen_PCD, SIGNAL(triggered()), this, SLOT(slot_IO_OpenFilePCD()));
}

Display_Result_Qt::~Display_Result_Qt()
{

}

void Display_Result_Qt::slot_IO_OpenFilePCD()
{

}