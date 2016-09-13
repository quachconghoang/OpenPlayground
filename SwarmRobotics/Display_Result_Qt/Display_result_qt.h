#ifndef DISPLAY_RESULT_QT_H
#define DISPLAY_RESULT_QT_H

#include <QtWidgets/QMainWindow>
#include "ui_Display_Result_Qt.h"
#include "PCLViewer.h"
#include "ProcessBox.h"
#include "PCLStorage.h"
#include "VisualConnector.h"

class Display_Result_Qt : public QMainWindow
{
	Q_OBJECT

public:
	Display_Result_Qt(QWidget *parent = 0);
	~Display_Result_Qt();

private:

	Ui::Display_Result_QtClass ui;

	ProcessBox * processBox;
	ProcessBox * processBoxWaypoints;
	QProgressDialog * warningDialog;

	PCLViewer m_viewer3D;
	PCLStorage m_cloudStorage;

	VisualConnector * visualConnector;
	std::vector<double> processParam;

	void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

	private slots:
	void slot_IO_OpenFilePCD();

	void slot_Processing_Segment(std::vector<double> values);

	void slot_Processing_CapturePoints(std::vector<double> values);

	void slot_UI_BarUpdate(int val);
	void slot_UI_Finish(PS_WORKING_MODE mode);
	void slot_UI_PlaneChanged(bool isSelected);

};

#endif // DISPLAY_RESULT_QT_H
