#include "stdafx.h"
#include "PCLStorage.h"
#include "pcl/common/common_headers.h"


PCLStorage::PCLStorage()
{
}


PCLStorage::~PCLStorage()
{
}

void PCLStorage::setInputCloud(std::string fileName)
{
	cloud_input.reset(new PointCloudT);
	pcl::io::loadPCDFile(fileName, *cloud_input);
}