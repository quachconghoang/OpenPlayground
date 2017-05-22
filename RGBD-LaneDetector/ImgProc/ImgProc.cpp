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
	cv::threshold(tmpResult, tmpResult, 0.8, 1, CV_THRESH_BINARY);

	cv::Rect rsRect = cv::Rect(0, 0, tmpResult.cols, tmpResult.rows);
	cv::Point jumpDistance(jumpStep * fabs(pca_rs[0]), jumpStep * fabs(pca_rs[1]));

	listPoints.push_back(initPoint);

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
			boxTL += tmpCenter;
			boxBR = boxTL + jumpDistance;
		}
	}
	std::reverse(listPoints.begin(), listPoints.end());

	//Go Up --||--
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
			boxTL -= tmpCenter;
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

cv::Vec4f fast_PlaneDetect(const cv::Mat & depthMat, cv::Mat & rgbMat, const ImgProc3D::Intr & camInfo, bool img320x240/* =false */)
{
	cv::RNG rng(0xFFFFFFFF);
	bool planeDetected = false;

	int imgW = depthMat.cols;
	int imgH = depthMat.rows;

	cv::Point2i invalid_loc, p1_loc, p2_loc, p3_loc;
	if (img320x240)	{
		p1_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2, imgH - 30), rng, cv::Point2i(50, 25));
		p2_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 - 80, imgH / 2), rng, cv::Point2i(50, 20));
		p3_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 + 80, imgH / 2), rng, cv::Point2i(50, 20));
	}
	else {
		p1_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2, imgH - 60), rng, cv::Point2i(50, 30));
		p2_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 - 150, imgH / 2), rng, cv::Point2i(50, 20));
		p3_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 + 150, imgH / 2), rng, cv::Point2i(50, 20));
	}

	if (p1_loc == INVALID_CVPOINT2i || p2_loc == INVALID_CVPOINT2i || p3_loc == INVALID_CVPOINT2i)
	{
		std::cout << "Detect plane failed \n";
		return cv::Vec4f(0.f, 1.f, 0.f, -0.03f); //Get previous values or something else
	}

	//cv::circle(rgbMat, p1_loc, 3, cv::Scalar(0, 0, 255), 3);
	//cv::circle(rgbMat, p2_loc, 3, cv::Scalar(0, 0, 255), 3);
	//cv::circle(rgbMat, p3_loc, 3, cv::Scalar(0, 0, 255), 3);

	cv::Vec3f p1 = getPointXYZ(depthMat, camInfo, p1_loc);
	cv::Vec3f p2 = getPointXYZ(depthMat, camInfo, p2_loc);
	cv::Vec3f p3 = getPointXYZ(depthMat, camInfo, p3_loc);
	cv::Vec3f c = (p1 + p2 + p3) / 3;

	cv::Vec3f v1 = cv::normalize(p2 - p1);
	cv::Vec3f v2 = cv::normalize(p3 - p1);

	cv::Vec3f planeNormal = cv::normalize(v1.cross(v2));
	float d = -planeNormal[0] * c[0] - planeNormal[1] * c[1] - planeNormal[2] * c[2];//d = - ax -by -cz;

	//printf("Model: %f *x + %f *y + %f *z + %f = 0\n", planeNormal[0],planeNormal[1],planeNormal[2],d);
	return cv::Vec4f(planeNormal[0], planeNormal[1], planeNormal[2], d);
}

void convertToXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Mat & xyzMat)
{
	cv::Mat depthImgF;
	depthMat.convertTo(depthImgF, CV_32F); // convert the image data to float type 
	depthImgF /= camInfo.scale;

	const float qnan = std::numeric_limits<float>::quiet_NaN();

	int idx = 0;
	for (int i = 0; i < depthImgF.rows; i++)
	{
		for (int j = 0; j < depthImgF.cols; j++)
		{
			float d = depthImgF.at<float>(i, j);
			

			if (d == 0)
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f(qnan, qnan, qnan);
			else
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f((j - camInfo.cx) * d / camInfo.fx, (i - camInfo.cy) * d / camInfo.fy, d);
			idx++;
		}
	}
}

void fillLaneMap2D(cv::Mat & xyzMat, cv::Mat & lane2D, cv::Vec4f planeModel, float threshold)
{
	cv::Point3f planeNormal(planeModel[0], planeModel[1], planeModel[2]);
	cv::Point3f e_1 = planeNormal.cross(cv::Point3f(0, 0, 1));
	cv::Point3f e_2 = -planeNormal.cross(e_1);
	cv::Point3f planeOrg = -planeModel[3] * planeNormal;

	for (int i = 0; i < xyzMat.rows; i++)
	{
		for (int j = 0; j < xyzMat.cols; j++)
		{
			cv::Point3f p = xyzMat.at<cv::Point3f>(i, j);
			if (!std::isnan(p.x))
			{
				if (fabs(planeModel[0] * p.x + planeModel[1] * p.y + planeModel[2] * p.z + planeModel[3]) < threshold)
				{
					cv::Point3f p_t = p - planeOrg;

					float p_x_new = p_t.dot(e_1);
					float p_y_new = p_t.dot(e_2);
					lane2D.at<cv::Point2f>(i, j) = cv::Point2f(p_x_new, p_y_new);
				}
			}
			/*	NEAR RANGE INTERPOLATION	*/
			else{
				if (i > 120){
					const float fx = 616.442444f;
					const float fy = 616.442444f;
					const float cx = 319.5f;
					const float cy = 239.5f;
					cv::Point3f rayLine = cv::normalize(cv::Vec3f((j - cx) / fx, (i - cy) / fy, 1));
					float r = rayLine.dot(planeNormal);

					p = (-planeModel[3]) / r * rayLine;
					cv::Point3f p_t = p - planeOrg;
					float p_x_new = p_t.dot(e_1);
					float p_y_new = p_t.dot(e_2);
					lane2D.at<cv::Point2f>(i, j) = cv::Point2f(p_x_new, p_y_new);
				}
			}
		}
	}
}

void create2DGrid(cv::Mat & lane2DMap, cv::Mat & colorMap, cv::Mat & gridMap)
{
	// 1 pixel = 2cm
	float scale = 1/0.02;
	int width = gridMap.cols;
	int height = gridMap.rows;
	cv::Rect mapRect(0, 0, width, height);

	for (int i = 0; i < lane2DMap.rows; i++)
	{
		for (int j = 0; j < lane2DMap.cols; j++)
		{
			cv::Point2f p = lane2DMap.at<cv::Point2f>(i, j);
			if (!std::isnan(p.x))
			{
				int new_x = int(width / 2 + p.x * scale);
				int new_y = int(height - p.y * scale);
				if (mapRect.contains(cv::Point2i(new_x, new_y)))
				{
					gridMap.at<cv::Vec3b>(new_y, new_x) = colorMap.at<cv::Vec3b>(i, j);
				}
			}
		}
	}
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