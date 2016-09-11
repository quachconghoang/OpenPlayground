#ifndef DISPLAY_RESULT_QT_H
#define DISPLAY_RESULT_QT_H

#include <QtWidgets/QMainWindow>
#include "ui_Display_Result_Qt.h"
#include "PCLViewer.h"
#include "ProcessBox.h"

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

	PCLViewer viewer3D;
	PCLStorage cloudStorage;

	private slots:
	void slot_IO_OpenFilePCD();

	void slot_Processing_Segment(std::vector<double> values);

	void slot_Processing_CapturePoints(std::vector<double> values);

};

#endif // DISPLAY_RESULT_QT_H
