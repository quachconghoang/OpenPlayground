#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);

void removeUncertain(const cv::Mat depthImg, cv::Mat & rgb)
{
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			ushort realDepth = depthImg.at<ushort>(i, j);

			if (realDepth == 0 || realDepth > 60000)
			{
				rgb.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
			}
		}
	}
}

int main()
{
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Realsense_RAW);
	//cv::viz::Viz3d viz("show_cloud");

	cv::Mat img = cv::imread("../Data/2516-rgb.png");
	cv::Mat dimg = cv::imread("../Data/2516-d.png", CV_LOAD_IMAGE_ANYDEPTH);

	cv::cuda::GpuMat dev_dMat(dimg);
	cv::cuda::GpuMat dev_rgbMat(img);

	cv::cuda::GpuMat dev_xyzMap(dimg.rows, dimg.cols, CV_32FC3);
	cv::cuda::GpuMat dev_normalMap(dimg.rows, dimg.cols, CV_32FC3);
	cv::cuda::GpuMat dev_objectMap(dimg.rows, dimg.cols, CV_8UC1);
	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	ImgProc3D::convertTo_NormalsMap(dev_xyzMap, dev_normalMap);

	cv::Mat xyzMap, laneMap2D, normalMap, objMap;
	dev_normalMap.download(normalMap);
	dev_xyzMap.download(xyzMap);

	cv::Mat depthVizMat = cv::Mat(480, 640, CV_8UC3);
	colorizeDepthImage(dimg, depthVizMat);
	cv::imshow("depth IMG", depthVizMat);

	
	cv::imshow("RGB", img);
	removeUncertain(dimg, img);
	cv::imshow("RGB-ux", img);
	cv::waitKey();

	//PointCloudXYZRGB cloudXYZrgb;
	//convertToPointcloud(img, dimg, intr, cloudXYZrgb);
	//viz.showWidget("w", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors)/*, cvaff*/);
	//viz.showWidget("coo-sys", cv::viz::WCoordinateSystem(5.0));
	//viz.spin();
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