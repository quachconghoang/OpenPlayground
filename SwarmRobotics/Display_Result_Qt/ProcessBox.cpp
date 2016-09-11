#include "stdafx.h"
#include "ProcessBox.h"

ProcessBox::ProcessBox(QWidget *parent)
	: QDialog(parent),mode(PMODE_SEGMENTATION)
{
	ui.setupUi(this);
	connect(ui.buttonBox,SIGNAL( accepted() ), this, SLOT( pushAccepted() ) );
	connect(ui.buttonBox,SIGNAL( rejected() ), this, SLOT( pushRejected() ) );
}

ProcessBox::~ProcessBox()
{

}

void ProcessBox::setProcessMode(PROCESS_MODE processMode)
{
	mode = processMode;
	switch (processMode)
	{
	case PMODE_SEGMENTATION:
		//this->resize(300,90);
		setWindowTitle(QString("Segmentation Params"));
		ui.param1SpinBox->setMaximum(30);ui.param1SpinBox->setMinimum(0.1);ui.param1SpinBox->setValue(2.0);ui.param1SpinBox->setSingleStep(0.1);
		ui.param1Label->setText(QString("Min plane area:"));ui.param1Unit->setText(QString("m2"));

		ui.param2SpinBox->setMaximum(1000);ui.param2SpinBox->setMinimum(50);ui.param2SpinBox->setValue(200);ui.param2SpinBox->setSingleStep(1);
		ui.param2Label->setText(QString("Distance Threshold:"));ui.param2Unit->setText(QString("millimeter"));

		ui.param3SpinBox->setMaximum(1000);ui.param3SpinBox->setMinimum(30);ui.param3SpinBox->setValue(200);ui.param3SpinBox->setSingleStep(1);
		ui.param3Label->setText(QString("Max iterations:"));ui.param3Unit->setText(QString("iterations"));

		ui.param4SpinBox->setMaximum(1000);ui.param4SpinBox->setMinimum(50);ui.param4SpinBox->setValue(200);ui.param4SpinBox->setSingleStep(1);
		ui.param4Label->setText(QString("Cluster threshold:"));ui.param4Unit->setText(QString("millimeter"));

		//ui.param2Label->setVisible(false);ui.param2Unit->setVisible(false);ui.param2SpinBox->setVisible(false);
		//ui.param3Label->setVisible(false);ui.param3Unit->setVisible(false);ui.param3SpinBox->setVisible(false);
		//ui.buttonBox->setGeometry(90,50,190,40);
		break;
	case PMODE_WAYPOINTS:
		setWindowTitle(QString("Capture points Params"));
		ui.param1SpinBox->setMaximum(2000);ui.param1SpinBox->setMinimum(-2000);ui.param1SpinBox->setValue(800);ui.param1SpinBox->setSingleStep(1);
		ui.param1Label->setText(QString("Attitude:"));ui.param1Unit->setText(QString("millimeter"));

		ui.param2SpinBox->setMaximum(5000);ui.param2SpinBox->setMinimum(300);ui.param2SpinBox->setValue(600);ui.param2SpinBox->setSingleStep(1);
		ui.param2Label->setText(QString("Width:"));ui.param2Unit->setText(QString("millimeter"));

		ui.param3SpinBox->setMaximum(5000);ui.param3SpinBox->setMinimum(300);ui.param3SpinBox->setValue(400);ui.param3SpinBox->setSingleStep(1);
		ui.param3Label->setText(QString("Height:"));ui.param3Unit->setText(QString("millimeter"));

		this->resize(320,180);
		ui.param4Label->setVisible(false);ui.param4Unit->setVisible(false);ui.param4SpinBox->setVisible(false);
		ui.buttonBox->setGeometry(110,135,190,40);
		break;
	default:
		break;
	}
}

void ProcessBox::pushAccepted()
{
	std::vector<double> res;
	switch (mode)
	{
	case PMODE_SEGMENTATION:
		res.push_back(ui.param1SpinBox->value());
		res.push_back(ui.param2SpinBox->value());
		res.push_back(ui.param3SpinBox->value());
		res.push_back(ui.param4SpinBox->value());
		break;
	case PMODE_WAYPOINTS:
		res.push_back(ui.param1SpinBox->value());
		res.push_back(ui.param2SpinBox->value());
		res.push_back(ui.param3SpinBox->value());
		break;
	default:
		break;
	}
	emit this->process(res);
}

void ProcessBox::pushRejected()
{
	std::vector<double> res;
	res.push_back(PROCESSBOX_CANCEL);
	emit this->process(res);
}