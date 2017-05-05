#include "LaneDetector.h"
#include "opencv2/opencv.hpp"

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/pcl_base.h>
//
//void getPlaneModel(){}


#ifndef LANE_MAP_SIZE
#define LANE_MAP_SIZE 512
#endif

#define TEMPLATE_SIZE 32

static float qnan = std::numeric_limits<float>::quiet_NaN();

ImgProc3D::LaneDetector::LaneDetector(int imgWidth, int imgHeight)
{
	xyzMap = cv::Mat(imgHeight, imgWidth, CV_32FC3);
	laneProject2D = cv::Mat(imgHeight, imgWidth, CV_32FC2, cv::Scalar(qnan, qnan));
	laneMap2D = cv::Mat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
	dev_dMat = cv::cuda::GpuMat(imgHeight,imgWidth,CV_16UC1);
	dev_rgbMat = cv::cuda::GpuMat(imgHeight, imgWidth, CV_8UC3);
	dev_xyzMap = cv::cuda::GpuMat(imgHeight, imgWidth, CV_32FC3);
	dev_laneMap2D = cv::cuda::GpuMat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));

	
	setCamera(ImgProc3D::IntrMode_Realsense_RAW);
	generateTemplate();
}

ImgProc3D::LaneDetector::~LaneDetector()
{
	
}

void ImgProc3D::LaneDetector::setCamera(ImgProc3D::IntrMode _mode)
{
	m_camInfo = ImgProc3D::Intr(_mode);
}

void ImgProc3D::LaneDetector::generateTemplate()
{
	templateSize = cv::Size(TEMPLATE_SIZE, TEMPLATE_SIZE);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	cv::line(tmp_right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 8);
	cv::line(tmp_left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 8);
}

void templateProcessing(cv::Mat & imgBin, const cv::Mat &tmp, cv::Mat & matchingResult)
{
	int match_method = CV_TM_CCOEFF_NORMED;
	int result_cols = imgBin.cols - tmp.cols + 1;
	int result_rows = imgBin.rows - tmp.rows + 1;
	matchingResult.create(imgBin.rows, imgBin.cols, CV_32FC1);
	cv::matchTemplate(imgBin, tmp, matchingResult, match_method);

	//cv::GaussianBlur(matchingResult, matchingResult, cv::Size(5, 5), 0, 0);
	//cv::normalize(matchingResult, matchingResult, 0, 1, cv::NORM_MINMAX, -1);
}

cv::Point getBestMatchLoc(const cv::Mat & matchingResult, double & maxVal)
{
	double minVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(matchingResult, &minVal, &maxVal, &minLoc, &maxLoc);
	return maxLoc;
}

void fillPointCloud(cv::Mat & xyz, cv::Rect search_rect, std::vector<cv::Point3f> & cloud)
{
	for (int i = search_rect.tl().y; i < search_rect.br().y; i++)
	{
		for (int j = search_rect.tl().x; j < search_rect.br().x; j++)
		{
			cv::Point3f cPoint = xyz.at<cv::Point3f>(i, j);
			if (!std::isnan(cPoint.z) && cPoint.z < 10)
			{
				cloud.push_back(cPoint);
			}
		}
	}
}

void fillRegion(cv::Mat & dMat, cv::Mat & rgbMat, cv::Rect search_rect)
{
	for (int i = 0; i < search_rect.height; i++)
	{
		for (int j = 0; j < search_rect.width; j++)
		{
			cv::Point c_point = search_rect.tl() + cv::Point(j, i);
			ushort realDepth = dMat.at<ushort>(c_point);
			if (realDepth != 0 && realDepth < 50000)	
			{
				rgbMat.at<cv::Vec3b>(c_point)[2] = 255;
			}
		}
	}
}

cv::Rect safeCreateRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedSize)
{
	int safeWidth = preferedSize.width;
	int safeHeight = preferedSize.height;
	if (safeWidth + tl_point.x > imgSize.width) 
		safeWidth = imgSize.width - tl_point.x;
	if (safeHeight + tl_point.y > imgSize.height) 
		safeHeight = imgSize.height - tl_point.y;

	return cv::Rect(tl_point, cv::Size(safeWidth,safeHeight));
}

void ImgProc3D::LaneDetector::processFrame(cv::Mat & raw_img, cv::Mat & raw_dimg)
{
	//CONVERT TO XYZ & NORMAL
	currentColorMat = raw_img.clone();
	dev_dMat.upload(raw_dimg);
	dev_rgbMat.upload(raw_img);
	cv::Mat normalMap, objMap;

	cv::cuda::GpuMat dev_normalMap(raw_dimg.rows, raw_dimg.cols, CV_32FC3);
	ImgProc3D::convertTo_NormalsMap(dev_xyzMap, dev_normalMap);
	dev_normalMap.download(normalMap);
	
	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	dev_xyzMap.download(xyzMap);

	//FIND PLANE-MODEL
	samplePoints.clear();
	cv::Point op(0, 80);
	cv::Rect cropRect(op.x, op.y, 640, 320);
	cv::Mat rgbimg = raw_img(cropRect);
	cv::Mat imgGray, imgBin;
	cv::cvtColor(rgbimg, imgGray, cv::COLOR_RGB2GRAY);
	cv::threshold(imgGray, imgBin, 150, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO

	cv::imshow("imgGray", imgGray);
	cv::imshow("imgBin", imgBin);

	cv::Mat result_left, result_right, result_prev;
	templateProcessing(imgBin, tmp_left, result_left);
	templateProcessing(imgBin, tmp_right, result_right);

	double maxVal_Left, maxVal_Right;
	cv::Point matchLocLeft = getBestMatchLoc(result_left, maxVal_Left) + op;
	cv::Point matchLocRight = getBestMatchLoc(result_right, maxVal_Right) + op;

	cv::Rect tmpRect_Left = safeCreateRect(matchLocLeft, cv::Size(640, 480), templateSize);//cv::Rect(matchLocLeft, cv::Size(32, 32));
	cv::Rect tmpRect_Right = safeCreateRect(matchLocRight, cv::Size(640, 480), templateSize);

	//fillRegion(dimg, rgbimg, tmpRect_Left);
	//fillRegion(dimg, rgbimg, tmpRect_Right);

	cv::rectangle(raw_img, tmpRect_Left, cv::Scalar(0, 255, 0), 2);
	cv::rectangle(raw_img, tmpRect_Right, cv::Scalar(255, 0, 0), 2);
	fillRegion(raw_dimg, raw_img, tmpRect_Left);
	fillRegion(raw_dimg, raw_img, tmpRect_Right);

	fillPointCloud(xyzMap, tmpRect_Left, samplePoints);
	fillPointCloud(xyzMap, tmpRect_Right, samplePoints);

	cv::putText(raw_img, std::to_string(maxVal_Left), matchLocLeft - cv::Point(20, 5), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
	cv::putText(raw_img, std::to_string(maxVal_Right), matchLocRight - cv::Point(20, 5), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

}

void ImgProc3D::LaneDetector::genarateMap2D(cv::Vec4f pmodel)
{
	laneProject2D.setTo(cv::Scalar(qnan, qnan));
	laneMap2D.setTo(cv::Scalar(0, 0, 0));
	fillLaneMap2D(xyzMap, laneProject2D, pmodel, 1.5);
	create2DGrid(laneProject2D, currentColorMat, laneMap2D);
	cv::imshow("Lane",laneMap2D);
	std::cout << "Done";
}