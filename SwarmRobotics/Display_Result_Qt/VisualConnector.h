#ifndef VISUALCONNECTOR_H
#define VISUALCONNECTOR_H

#include <QObject>

enum PS_WORKING_MODE
{
	PSWM_LOAD_POINTCLOUD = 0,
	PSWM_LOAD_WORKSPACE = 1,
	PSWM_SAVE_WORKSPACE = 2,
	PSWM_NEW_WAYPOINT = 3,
	PSWM_SEGMENT_DONE = 4
};

/** \brief  Class for connect pcl visualization module with QT Signals / slots.
  * \author HoangQC
  * \note This class was created for avoiding macro conflict with QT and Boost (pcl dependence)
  * \ingroup UAV-Visualization
  */
class VisualConnector : public QObject
{
	Q_OBJECT

public:
	VisualConnector(QObject *parent);
	~VisualConnector();

	/** \brief  Receive plane selected status form ProcessStorage then emit signals to QT Gui.*/
	void planeDidSelected(bool isSelected);

signals:
	void signal_planeDidSelected(bool isSelected);
	void signal_processFinish(PS_WORKING_MODE mode);
	void signal_processBarUpdating(int val);

private:
	
};

#endif // VISUALCONNECTOR_H
