#ifndef PROCESSBOX_H
#define PROCESSBOX_H

#include <QDialog>
#include "ui_ProcessBox.h"

/** \brief Type of process dialog.
  * \author HoangQC
  * \ingroup QT-GUI
  */
enum PROCESS_MODE
{
	PMODE_SEGMENTATION,
	PMODE_WAYPOINTS
};

#define  PROCESSBOX_CANCEL -999999

class ProcessBox : public QDialog
{
	Q_OBJECT

public:
	ProcessBox(QWidget *parent = 0);
	~ProcessBox();
	PROCESS_MODE mode;

	/** \brief Setup displaying mode. */
	void setProcessMode(PROCESS_MODE processMode);
private:
	Ui::ProcessBox ui;

private slots:
	void pushAccepted();
	void pushRejected();

signals:
	void process(std::vector<double> params);
	void canceled();
};

#endif // PROCESSBOX_H
