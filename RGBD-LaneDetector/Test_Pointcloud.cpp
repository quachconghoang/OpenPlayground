#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

#include "opencv2/rgbd.hpp"

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646

std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-SUMMER/";
//std::string dirPath = "D:/LaneData/Sample_30-04/";
int count = 100;

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);
void depthMask(const cv::Mat & depthImg, cv::Mat & mask);

#define IMG_WIDTH 640
#define IMG_HEIGHT 380

int main()
{
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Synthia_RGBD_HALF);
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	cv::viz::Viz3d viz("show_cloud");
	viz.showWidget("coo-sys", cv::viz::WCoordinateSystem());
	/// Let's assume camera has the following properties
	cv::Vec3f cam_pos(0.0f, 0.0f, 0.0f), cam_focal_point(0.0f, 0.0f, 2.0f), cam_y_dir(0.0f, 1.0f, 0.0f);
	cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	viz.setViewerPose(cam_pose);

	cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << m_camInfo.fx, 0, m_camInfo.cx, 0, m_camInfo.fy, m_camInfo.cy, 0, 0, 1);
	cv::rgbd::RgbdNormals n_estimate(IMG_HEIGHT, IMG_WIDTH, CV_32F, cameraMatrix, 5, cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_SRI);
	//cameraMatrix<float> << ;
	//cameraMatrix.at<float>(0, 0) = 0;

	while (count < dataHeaders.size()-1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);

		cv::resize(img, img, cv::Size(IMG_WIDTH, IMG_HEIGHT), 0, 0, cv::INTER_NEAREST);
		cv::resize(dimg, dimg, cv::Size(IMG_WIDTH, IMG_HEIGHT), 0, 0, cv::INTER_NEAREST);

		cv::cuda::GpuMat dev_dMat(dimg);
		cv::cuda::GpuMat dev_rgbMat(img);

		cv::cuda::GpuMat dev_xyzMap(dimg.rows, dimg.cols, CV_32FC3);
		//cv::cuda::GpuMat dev_normalMap(dimg.rows, dimg.cols, CV_32FC3);
		//cv::cuda::GpuMat dev_objectMap(dimg.rows, dimg.cols, CV_8UC1);
		ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
		//ImgProc3D::convertTo_NormalsMap(dev_xyzMap, dev_normalMap);
		//dev_normalMap.download(normalMap);

		cv::Mat xyzMap, laneMap2D, normalMap, objMap;
		
		dev_xyzMap.download(xyzMap);

		n_estimate(xyzMap, normalMap);
		//cv::rgbd::depthTo3d(dimg * 10, cameraMatrix, xyzMap);
		

		viz.showWidget("w", cv::viz::WCloud(xyzMap, img, normalMap)/*, cvaff*/);
		viz.spinOnce(1, true);

		
		//cv::putText(img, std::to_string(count), cv::Point(550, 400), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		cv::imshow("normal", normalMap);
		cv::imshow("RGB", img);
		cv::imshow("depth", dimg);
		/*cv::Mat depthVizMat = cv::Mat(480, 640, CV_8UC3);
		colorizeDepthImage(dimg, depthVizMat);*/

		cv::waitKey();
		count++;
	}
	


}

void depthMask(const cv::Mat & depthImg, cv::Mat & mask)
{
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			ushort realDepth = depthImg.at<ushort>(i, j);
			
			if (realDepth == 0 || realDepth > 25000)
			{
				mask.at<cv::Vec3b>(i, j) = cv::Vec3b(0,0,0);
			}
		}
	}
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