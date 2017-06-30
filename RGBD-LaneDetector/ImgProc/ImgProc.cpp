#include "ImgProc.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>

#define INVALID_CVPOINT2i cv::Point2i(-1, -1)

cv::Rect createSafeRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedRectSize)
{
	int safeWidth = preferedRectSize.width;
	int safeHeight = preferedRectSize.height;
	if (tl_point.x < 0) tl_point.x = 0;
	if (tl_point.y < 0) tl_point.y = 0;

	if (safeWidth + tl_point.x > imgSize.width)		safeWidth = imgSize.width - tl_point.x;
	if (safeHeight + tl_point.y > imgSize.height)	safeHeight = imgSize.height - tl_point.y;
	return cv::Rect(tl_point, cv::Size(safeWidth, safeHeight));
}

cv::Vec2f getAnglePCA(cv::Mat & tmpRS)
{
	//cv::normalize(tmpRS, tmpRS, 0, 1, cv::NORM_MINMAX, -1);
	std::vector<cv::Point2d> pcaData;
	for (int i = 0; i < tmpRS.rows; i++)
	{
		for (int j = 0; j < tmpRS.cols; j++)
		{
			if (tmpRS.at<float>(i, j) > 0.8f)
			{
				pcaData.push_back(cv::Point2d(j, i));
			}
		}
	}

	cv::Mat data_pts = cv::Mat(pcaData.size(), 2, CV_64FC1, &pcaData[0]);
	//std::cout << data_pts;

	//Perform PCA analysis
	cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
	//Store the center of the object
	cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
		static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
	//Store the eigenvalues and eigenvectors
	std::vector<cv::Point2d> eigen_vecs(2);
	std::vector<double> eigen_val(2);
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
	}
	//std::cout << eigen_vecs << " \n " << eigen_val[0] << "-" << eigen_val[1] <<"\n == \n";
	if (eigen_vecs[0].y<0)
	{
		eigen_vecs[0].x *= (-1);
		eigen_vecs[0].y *= (-1);
	}
	return cv::Vec2f(eigen_vecs[0].x, eigen_vecs[0].y);
}

void getAnglePCA(cv::Mat & tmpRS_normalized, PCA_Result & tRS)
{
	cv::normalize(tmpRS_normalized, tmpRS_normalized, 0, 1, cv::NORM_MINMAX, -1);
	std::vector<cv::Point2d> pcaData;
	for (int i = 0; i < tmpRS_normalized.rows; i++){
		for (int j = 0; j < tmpRS_normalized.cols; j++){
			if (tmpRS_normalized.at<float>(i, j) > 0.8f){
				pcaData.push_back(cv::Point2d(j, i));
			}
		}
	}

	if (pcaData.size() < 5) return;

	//Perform PCA analysis
	cv::Mat data_pts = cv::Mat(pcaData.size(), 2, CV_64FC1, &pcaData[0]);
	cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

	//Store the center of the object
	cv::Point cntr = cv::Point(
		static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
		static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
	
	std::vector<cv::Point2d> eigen_vecs(2);
	std::vector<double> eigen_val(2);
	
	for (int i = 0; i < 2; ++i)	{
		eigen_vecs[i] = cv::Point2d(
			pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
	}
	//std::cout << eigen_vecs << " \n " << eigen_val[0] << "-" << eigen_val[1] <<"\n == \n";
	if (eigen_vecs[0].y < 0)	{
		eigen_vecs[0].x *= (-1);
		eigen_vecs[0].y *= (-1);
	} //ROTATE

	tRS._vec = cv::Vec2f(eigen_vecs[0].x, eigen_vecs[0].y);
	tRS._val_1 = eigen_val[0];
	tRS._val_2 = eigen_val[1];
}

int countNonZeroCenter(cv::Mat & _map, cv::Point & center)
{
	center = cv::Point(0, 0);
	//cv::imshow("MapTMP", _map);
	//cv::waitKey();
	int count = 0;
	for (int i = 0; i < _map.rows; i++){
		for (int j = 0; j < _map.cols; j++){
			if (_map.at<float>(i, j) > 0.1){
				count++;
				center += cv::Point(j, i);
			}
		}
	}
	if (count != 0){
		center /= count;
	}
	return count;
}

void getLinePoints_SlindingBox_(cv::Mat & tmpResult, 
	std::vector<cv::Point> & listPoints, 
	cv::Point initPoint, cv::Vec2f pca_rs, 
	cv::Size boxSize /* = cv::Size(32,32) */, int jumpStep /* = 32 */)
{
	//cv::normalize(tmpResult, tmpResult, 0, 1, cv::NORM_MINMAX, -1);
	cv::threshold(tmpResult, tmpResult, 0.4, 1, CV_THRESH_BINARY);

	cv::Rect rsRect = cv::Rect(0, 0, tmpResult.cols, tmpResult.rows);
	cv::Point jumpDistance(jumpStep * fabs(pca_rs[0]), jumpStep * fabs(pca_rs[1]));

	listPoints.push_back(initPoint);
	initPoint -= cv::Point(boxSize.width / 2, boxSize.height/2);

	//Go Down --||--
	cv::Point boxTL = initPoint;
	cv::Point boxBR = initPoint + cv::Point(jumpStep, jumpStep);
	while (rsRect.contains(boxBR) && rsRect.contains(boxTL))
	{
		cv::Rect newBox(boxTL, boxBR);
		cv::Point tmpCenter;
		int nonZero = countNonZeroCenter(tmpResult(newBox), tmpCenter);
		if (nonZero < 20) { break; }
		else{
			listPoints.push_back(tmpCenter + boxTL);
			if (tmpCenter.x < 5 || tmpCenter.x < 5){
				tmpCenter += cv::Point(5, 5);
			}
			boxTL += tmpCenter;
			boxBR = boxTL + jumpDistance;
		}
	}
	std::reverse(listPoints.begin(), listPoints.end());

	//Go Up --||--
	int try_count = 0;
	boxTL = initPoint - jumpDistance;
	boxBR = initPoint;
	while (rsRect.contains(boxBR) && rsRect.contains(boxTL))
	{
		cv::Rect newBox(boxTL, boxBR);
		cv::Point tmpCenter;
		int nonZero = countNonZeroCenter(tmpResult(newBox), tmpCenter);
		if (nonZero < 20){
			break;		
		}
		else{
			listPoints.push_back(tmpCenter + boxTL);
			if (tmpCenter.x > boxSize.width || tmpCenter.x > boxSize.height){
				tmpCenter -= cv::Point(5, 5);
			}

			boxTL -= (cv::Point(boxSize.width, boxSize.height) - tmpCenter);
			boxBR = boxTL + jumpDistance;
		}
	}
}

cv::Point2i getRandomSample(const cv::Mat & depthMat, const cv::Point2i & origin, cv::RNG & rng, const cv::Point2i delta)
{
	cv::Point2i ranPoint;
	int tryCount = 0;
	do {
		ranPoint = cv::Point2i(
			rng.uniform(origin.x - delta.x, origin.x + delta.x),
			rng.uniform(origin.y - delta.y, origin.y + delta.y));
		tryCount++;
		if (tryCount == 100)
		{
			ranPoint = INVALID_CVPOINT2i;
			break;
		}
	} while (depthMat.at<ushort>(ranPoint) == 0);

	return ranPoint;
}

cv::Point3f getPointXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Point2i p)
{
	float d = depthMat.at<ushort>(p) / camInfo.scale;
	cv::Point3f rs = cv::Point3f(0, 0, 0);
	
	if (d!=0){
		rs = cv::Point3f((p.x - camInfo.cx) * d / camInfo.fx, 
			(p.y-camInfo.cy) * d / camInfo.fy,	d);
	}

	return rs;
}

cv::Point cuda_findMinmax(const cv::cuda::GpuMat & matchingResult, double & maxVal)
{
	double minVal;
	cv::Point minLoc, maxLoc;
	cv::cuda::minMaxLoc(matchingResult, &minVal, &maxVal, &minLoc, &maxLoc);
	return maxLoc;
}

cv::Point cpu_findMinmax(const cv::Mat & matchingResult, double & maxVal)
{
	double minVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(matchingResult, &minVal, &maxVal, &minLoc, &maxLoc);
	return maxLoc;
}


cv::Point searchPoint(const cv::Mat & dMat, cv::Point & origin, int searchRadius, cv::RNG & rng)
{
	int tryCount = 0;
	cv::Point2i ranPoint;
	do {
		ranPoint = cv::Point2i(
			rng.uniform(origin.x - searchRadius, origin.x + searchRadius),
			rng.uniform(origin.y - searchRadius, origin.y + searchRadius));
		tryCount++;
		if (tryCount == 30)
		{
			ranPoint = INVALID_CVPOINT2i;
			break;
		}
	} while (dMat.at<ushort>(ranPoint) == 0);

	return ranPoint;
}

cv::Vec4f getPlaneModel(const cv::Mat & dMat, const ImgProc3D::Intr & camInfo, cv::Point & leftPoint,
	cv::Point & rightPointNear, cv::Point & rightPointFar)
{
	cv::RNG rng(0xFFFFFFFF);

	cv::Point p1_loc = searchPoint(dMat, rightPointNear, 5, rng);
	cv::Point p2_loc = searchPoint(dMat, rightPointFar, 5, rng);
	cv::Point p3_loc = searchPoint(dMat, leftPoint, 5, rng);

	if (p3_loc == INVALID_CVPOINT2i || p2_loc == INVALID_CVPOINT2i || p3_loc == INVALID_CVPOINT2i)
	{
		std::cout << "Detect plane failed \n";
	}

	cv::Vec3f p1 = getPointXYZ(dMat, camInfo, p1_loc);
	cv::Vec3f p2 = getPointXYZ(dMat, camInfo, p2_loc);
	cv::Vec3f p3 = getPointXYZ(dMat, camInfo, p3_loc);

	cv::Vec3f v1 = cv::normalize(p2 - p1);
	cv::Vec3f v2 = cv::normalize(p3 - p1);

	cv::Vec3f planeNormal = cv::normalize(v1.cross(v2));
	float d = -planeNormal[0] * p1[0] - planeNormal[1] * p1[1] - planeNormal[2] * p1[2];//d = - ax -by -cz;
	printf("Model: %f *x + %f *y + %f *z + %f = 0\n", planeNormal[0], planeNormal[1], planeNormal[2], d);
	return cv::Vec4f(planeNormal[0], planeNormal[1], planeNormal[2], d);
}