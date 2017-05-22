#ifndef __IMG_PROC_H
#define __IMG_PROC_H

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include "types.h"

cv::Vec4f fast_PlaneDetect(const cv::Mat & depthMat, cv::Mat & rgbMat, const ImgProc3D::Intr & camInfo, bool img320x240=false);

//CPU CODE
void convertToXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Mat & xyzMat);
void fillLaneMap2D(cv::Mat & xyzMat, cv::Mat & lane2D, cv::Vec4f planeModel, float threshold=0.05f);
void create2DGrid(cv::Mat & lane2DMap, cv::Mat & colorMap, cv::Mat & gridMap);

cv::Point3f getPointXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Point2i p);

//Template matching utilities
cv::Rect createSafeRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedRectSize);
cv::Vec2f getAnglePCA(cv::Mat & tmpRS);
void getLinePoints_SlindingBox_(cv::Mat & tmpResult,
	std::vector<cv::Point> & listPoints, 
	cv::Point initPoint, cv::Vec2f pca_rs,
	cv::Size boxSize = cv::Size(32,32), 
	int jumpStep = 32);


int countNonZeroCenter(cv::Mat & _map, cv::Point & center);

#define INVALID_CVPOINT2i cv::Point2i(-1, -1)

cv::Point cpu_findMinmax(const cv::Mat & matchingResult, double & maxVal);

cv::Point cuda_findMinmax(const cv::cuda::GpuMat & matchingResult, double & maxVal);

#endif