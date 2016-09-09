#include "stdafx.h"
#include "Display_Result_Qt.h"

Display_Result_Qt::Display_Result_Qt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	viewer3D.setupPCLViewer(ui.qvtkWidget);

	connect(ui.actionOpen_PCD, SIGNAL(triggered()), this, SLOT(slot_IO_OpenFilePCD()));
}

Display_Result_Qt::~Display_Result_Qt()
{

}

void Display_Result_Qt::slot_IO_OpenFilePCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, QString("Open pointcloud file"), QString(".//Samples//"), QString("PCD file (*.pcd)"));
	if (fileName.length() > 0)
	{
		viewer3D.cloudStorage.setInputCloud(fileName.toStdString());
		viewer3D.displayRawData();
	}
}