#include "stdafx.h"
#include "Display_Result_Qt.h"

Display_Result_Qt::Display_Result_Qt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	viewer3D.setupPCLViewer(ui.qvtkWidget);

	connect(ui.actionOpen_PCD, SIGNAL(triggered()), this, SLOT(slot_IO_OpenFilePCD()));

	//connect(ui.actionSegmentation, SIGNAL(triggered()), this, SLOT(slot_Processing_Segment()));
	processBox = new ProcessBox(this);
	processBox->setProcessMode(PMODE_SEGMENTATION);
	connect(processBox, SIGNAL(process(std::vector<double>)), this, SLOT(slot_Processing_Segment(std::vector<double>)));
	connect(ui.actionSegmentation, SIGNAL(triggered()), processBox, SLOT(show()));
}

Display_Result_Qt::~Display_Result_Qt()
{

}

void Display_Result_Qt::slot_IO_OpenFilePCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, QString("Open pointcloud file"), QString(".//Samples//"), QString("PCD file (*.pcd)"));
	if (fileName.length() > 0)
	{
		cloudStorage.setInputCloud(fileName.toStdString());
		viewer3D.displayRawData(cloudStorage);
	}
}

void Display_Result_Qt::slot_Processing_Segment(std::vector<double> values)
{
	processBox->hide();
	if (values[0] != PROCESSBOX_CANCEL)
	{
		//cloudStorage.segmentPointcloud(values[0], values[1], values[2], values[3])
	}
	
}

void Display_Result_Qt::slot_Processing_CapturePoints(std::vector<double> values)
{
	qDebug() << "Setup - 2... ";
}