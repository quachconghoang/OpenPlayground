#ifndef DISPLAY_RESULT_QT_H
#define DISPLAY_RESULT_QT_H

#include <QtWidgets/QMainWindow>
#include "ui_Display_result_qt.h"

#include "pcl/common/common_headers.h"
#include "pcl/visualization/pcl_visualizer.h"

class Display_Result_Qt : public QMainWindow
{
	Q_OBJECT

public:
	Display_Result_Qt(QWidget *parent = 0);
	~Display_Result_Qt();

private:
	Ui::Display_Result_QtClass ui;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif // DISPLAY_RESULT_QT_H
