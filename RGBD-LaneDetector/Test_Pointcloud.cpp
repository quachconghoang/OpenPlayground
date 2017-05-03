#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646

std::string dirPath = "D:/LaneData/Sample_30-04/";
void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);

//1. Template matching
//2. Plane estimation
//3. Polyline fitting
cv::Mat tmp_left, tmp_right;

void generateTemplate()
{
	cv::Size templateSize(40, 40);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	cv::line(tmp_right, cv::Point(0, 0), cv::Point(40, 36), cv::Scalar(255), 8);
	cv::line(tmp_left, cv::Point(0, 36), cv::Point(40, 0), cv::Scalar(255), 8);

	cv::imshow("tmp-Left", tmp_left);
	cv::imshow("tmp-Right", tmp_right);
	cv::waitKey();
}

cv::Point templateMatching(cv::Mat &imgBin, const cv::Mat &tmp) {
	int match_method = CV_TM_CCOEFF_NORMED;
	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	cv::Mat result;
	int result_cols = imgBin.cols - tmp.cols + 1;
	int result_rows = imgBin.rows - tmp.rows + 1;
	result.create(imgBin.rows, imgBin.cols, CV_32FC1);
	cv::matchTemplate(imgBin, tmp, result, match_method);
	cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	cv::imshow("rs", result);
	cv::threshold(result, result, 0.8, 1, CV_THRESH_BINARY);
	cv::imshow("rs-bin", result);
	cv::waitKey();

	maxLoc.x += tmp.cols / 2;
	maxLoc.y += tmp.rows / 2;
	return maxLoc;
}

int main()
{
	generateTemplate();
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Realsense_RAW);
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	int count = 2000;
	//while (count < dataHeaders.size()-1)
	//{
	//	cv::Mat img = cv::imread(dirPath+dataHeaders[count].rgbImg);
	//	cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
	//	cv::imshow("depth IMG", dimg);
	//	cv::imshow("RGB", img(cv::Rect(0,60,640,420)));
	//	cv::waitKey(30);
	//	count++;
	//}

	cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
	cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
	cv::imshow("depth IMG", dimg);
	cv::imshow("RGB", img);

	cv::Mat imgGray, imgBin;
	cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
	cv::imshow("imgGray", imgGray);
	cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_BINARY | CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
	cv::imshow("imgBin", imgBin);
	cv::waitKey();

	cv::Mat imgBin_small;
	cv::resize(imgBin, imgBin_small, cv::Size(320, 240));

	cv::Point matchLocLeft = templateMatching(imgBin, tmp_left);
	cv::Point matchLocRight = templateMatching(imgBin, tmp_right);
	cv::Point matchLocMagic_Left = templateMatching(imgBin_small, tmp_left);
	cv::Point matchLocMagic_Right = templateMatching(imgBin_small, tmp_right);

	cv::circle(img, matchLocLeft, 4, cv::Scalar(0, 255, 0));
	cv::circle(img, matchLocRight, 4, cv::Scalar(255, 0, 0));

	cv::circle(img, matchLocMagic_Left * 2, 4, cv::Scalar(0, 0, 255));
	cv::circle(img, matchLocMagic_Right * 2, 4, cv::Scalar(0, 0, 255));
	cv::imshow("img", img);
	cv::waitKey();


	/*cv::cuda::GpuMat dev_dMat(dimg);
	cv::cuda::GpuMat dev_rgbMat(img);

	cv::cuda::GpuMat dev_xyzMap(dimg.rows, dimg.cols, CV_32FC3);
	cv::cuda::GpuMat dev_normalMap(dimg.rows, dimg.cols, CV_32FC3);
	cv::cuda::GpuMat dev_objectMap(dimg.rows, dimg.cols, CV_8UC1);
	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	ImgProc3D::convertTo_NormalsMap(dev_xyzMap, dev_normalMap);

	cv::Mat xyzMap, laneMap2D, normalMap, objMap;
	dev_normalMap.download(normalMap);
	dev_xyzMap.download(xyzMap);*/

	/*cv::Mat depthVizMat = cv::Mat(480, 640, CV_8UC3);
	colorizeDepthImage(dimg, depthVizMat);*/
}

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat)
{
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			ushort realDepth = depthImg.at<ushort>(i, j) / 5;
			int valCase = (realDepth - 50) / 256;
			uchar n = uchar((realDepth - 50) % 256);
			if (realDepth < 50) valCase = -1;

			switch (valCase){
			case 0:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, n, 255); break;
			case 1:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 255 - n); break;
			case 2:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(n, 255, 0); break;
			case 3:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255 - n, 0); break;
			case 4:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, n); break;
			case 5:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, n, 255); break;
			case 6:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255 - n); break;
			case 7:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255 - n, 0); break;
			case 8:		resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(255 - n, 0, 0); break;
			default:	resultMat.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);	break;
			}
		}
	}
}