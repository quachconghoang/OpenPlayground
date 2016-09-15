#include "stdafx.h"
#include "Display_Result_Qt.h"
#include <QtConcurrent/QtConcurrentRun>

Display_Result_Qt::Display_Result_Qt(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	m_viewer3D.setupPCLViewer(ui.qvtkWidget);
	m_viewer3D.pclStorage = &m_cloudStorage;

	connect(ui.actionOpen_PCD, SIGNAL(triggered()), this, SLOT(slot_IO_OpenFilePCD()));

	//connect(ui.actionSegmentation, SIGNAL(triggered()), this, SLOT(slot_Processing_Segment()));
	processBox = new ProcessBox(this);
	processBox->setProcessMode(PMODE_SEGMENTATION);
	connect(processBox, SIGNAL(process(std::vector<double>)), this, SLOT(slot_Processing_Segment(std::vector<double>)));
	connect(ui.actionSegmentation, SIGNAL(triggered()), processBox, SLOT(show()));

	connect(ui.actionShowOrigin, SIGNAL(triggered(bool)), this, SLOT(slot_UI_ShowOrigin(bool)));
	connect(ui.actionShowGridCloud, SIGNAL(triggered(bool)), this, SLOT(slot_UI_ShowGrid(bool)));
	connect(ui.actionShow_Mesh, SIGNAL(triggered(bool)), this, SLOT(slot_UI_ShowMesh(bool)));

	this->visualConnector = new VisualConnector(this);
	m_cloudStorage.visualConnector = this->visualConnector;
	connect(visualConnector, SIGNAL(signal_planeDidSelected(bool)), this, SLOT(slot_UI_PlaneChanged(bool)));
	qRegisterMetaType<PS_WORKING_MODE>("PS_WORKING_MODE");
	connect(visualConnector, SIGNAL(signal_processFinish(PS_WORKING_MODE)), this, SLOT(slot_UI_Finish(PS_WORKING_MODE)));
	connect(visualConnector, SIGNAL(signal_processBarUpdating(int)), this, SLOT(slot_UI_BarUpdate(int)));
}

Display_Result_Qt::~Display_Result_Qt()
{

}

void Display_Result_Qt::slot_IO_OpenFilePCD()
{
	QString fileName = QFileDialog::getOpenFileName(this, QString("Open pointcloud file"), QString(".//Samples//"), QString("PCD file (*.pcd)"));
	if (fileName.length() > 0)
	{
		m_cloudStorage.setInputCloud(fileName.toStdString());
		m_viewer3D.displayRawData(m_cloudStorage);
		ui.actionSegmentation->setEnabled(true);
		ui.actionShowOrigin->setEnabled(true);
		ui.actionShowOrigin->setChecked(true);
	}
}

void Display_Result_Qt::slot_Processing_Segment(std::vector<double> values)
{
	processBox->hide();
	if (values[0] != PROCESSBOX_CANCEL)
	{
		warningDialog = new QProgressDialog(this);
		warningDialog->setLabelText(QString("Progressing"));
		warningDialog->setCancelButton(0);
		warningDialog->setWindowFlags(Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
		warningDialog->setValue(0);
		warningDialog->show();
		processParam.clear();
		processParam = values;

		QtConcurrent::run(std::bind(&PCLStorage::segmentParams, &m_cloudStorage, processParam));
		
		ui.actionSegmentation->setEnabled(false);
		ui.actionShowGridCloud->setEnabled(true);
	}
	
}

void Display_Result_Qt::slot_Processing_CapturePoints(std::vector<double> values)
{
	qDebug() << "Setup - 2... ";
}

void Display_Result_Qt::slot_UI_ShowOrigin(bool isChecked)
{
	using namespace pcl::visualization;
	if (isChecked){	m_viewer3D.pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 1, m_cloudStorage.tagID);}
	else{ m_viewer3D.pclVisualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 0, m_cloudStorage.tagID); }
	ui.actionShowOrigin->setChecked(isChecked);
	m_viewer3D.qvtkWidget->update();
}

void Display_Result_Qt::slot_UI_ShowGrid(bool isChecked)
{
	if (isChecked){
		ui.actionShow_Mesh->setEnabled(true);
		m_viewer3D.unHighlightSurfaces(&m_cloudStorage);
		m_viewer3D.displayGridSurfaces(&m_cloudStorage);
	}
	else{
		ui.actionShow_Mesh->setChecked(false);
		ui.actionShow_Mesh->setEnabled(false);
		m_viewer3D.unHighlightSurfaces(&m_cloudStorage);
		m_viewer3D.hideMeshSurfaces(&m_cloudStorage);
		m_viewer3D.displaySurfaces(&m_cloudStorage);
	}
	m_viewer3D.qvtkWidget->update();
}

void Display_Result_Qt::slot_UI_ShowMesh(bool isChecked)
{
	if (isChecked)	m_viewer3D.displayMeshSurfaces(&m_cloudStorage);
	else			m_viewer3D.hideMeshSurfaces(&m_cloudStorage);
	m_viewer3D.qvtkWidget->update();
}

void Display_Result_Qt::slot_UI_BarUpdate(int val)
{
	warningDialog->setValue(val);
}

void Display_Result_Qt::slot_UI_Finish(PS_WORKING_MODE mode)
{
	switch (mode)
	{
	case PSWM_LOAD_POINTCLOUD:
		break;

	case PSWM_LOAD_WORKSPACE:
		break;

	case PSWM_SAVE_WORKSPACE:
		break;

	case  PSWM_NEW_WAYPOINT:
		/*m_viewer->displayCapturePoints(&m_storage, m_storage.selected_Index);
		m_viewer->unHighlightSurfaces(&m_storage);
		slot_refresh3DWidget();
		setCursor(Qt::ArrowCursor);
		if (!m_storage.pathFinder->isPathFound) QMessageBox::warning(this, tr("Warning"), tr("Could not found waypoints to cover whole surface"));*/
		break;

	case  PSWM_SEGMENT_DONE:
		m_viewer3D.displaySurfaces(&m_cloudStorage);
		warningDialog->hide();
		//m_viewer3D.qvtkWidget->update();
		//ui.actionDisplayPlanes->setEnabled(true);
		//ui.actionShow_Object->setEnabled(true);
		//ui.actionSave_workspace->setEnabled(true);
		break;

	default:
		break;
	}
	//warningDialog->hide();
}

void Display_Result_Qt::slot_UI_PlaneChanged(bool isSelected)
{

}