#include "stdafx.h"
#include "VisualConnector.h"

VisualConnector::VisualConnector(QObject *parent)
	: QObject(parent)
{

}

VisualConnector::~VisualConnector()
{

}

void VisualConnector::planeDidSelected(bool isSelected)
{
	emit this->signal_planeDidSelected(isSelected);
}