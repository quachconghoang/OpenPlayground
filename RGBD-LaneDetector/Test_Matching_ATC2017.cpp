#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"
#include "opencv2/rgbd.hpp"

int64 st, et;

#define LOG_START st=cv::getTickCount();
#define LOG_STOP printf("time = %f \n", (cv::getTickCount()-st)/cv::getTickFrequency());
#define CV_COLOR_RED cv::Scalar(0, 0, 255)
#define CV_COLOR_GREEN cv::Scalar(0, 255, 0)
#define CV_COLOR_BLUE cv::Scalar(255, 0, 0)

//#define DISPLAY_OLD_METHOD

using namespace cv;

cv::Size templateSize(32, 32);
cv::Point templateCenter(16, 16);

 //Synthetic data params
std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-FOG/";
int count = 400;
bool needResize = true;
cv::Size fullImg_Size(1280, 760);
cv::Size processImg_Size(640, 380);
cv::Rect laneRegion(0, 156, 640, 224);
cv::Point orgTemplate(0 + 16, 156 + 16);

// Real data params
//std::string dirPath = "D:/LaneData/Sample_30-04/";
//int count = 1800;
//bool needResize = false;
//cv::Size fullImg_Size(640, 480);
//cv::Size processImg_Size(640, 480);
//cv::Rect laneRegion(0, 100, 640, 380);
//cv::Point orgTemplate(0 + 16, 100 + 16);
//ImgProc3D::Intr m_camInfo(ImgProc3D::IntrMode_Realsense_RAW);

cv::Point toOriginal(cv::Point p)	{ return p + laneRegion.tl() + templateCenter; }
cv::Rect toOriginal(cv::Rect r)		{ return cv::Rect(r.tl() + laneRegion.tl() + templateCenter, r.size()); }
cv::Size matchResult_Size(
	laneRegion.width - templateSize.width + 1,
	laneRegion.height - templateSize.height + 1);

struct MatchingKernel
{
	cv::Mat left, right;
	cv::Mat left_enhance;
	cv::Mat right_enhance;
};

void generateTemplate(MatchingKernel & kernel)
{
	kernel.left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	kernel.right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	
	cv::line(kernel.left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	cv::line(kernel.right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);

}

void displayKernel(MatchingKernel & kernel)
{
	cv::Mat displayMat(88,88,CV_8UC1,128);
	kernel.left.copyTo(displayMat(cv::Rect(8, 8, 32, 32)));
	kernel.right.copyTo(displayMat(cv::Rect(48, 8, 32, 32)));
	cv::imshow("K", displayMat);
}

#define D_BETA 0.2f
#define D_GAMMA 0.2f
#define  depthThresh 1000
void compute_Geometric_Map(cv::Mat & dImage_Meter, cv::Mat & resultMap)
{
	resultMap = cv::Mat(matchResult_Size, CV_32FC1, cv::Scalar(0));
	for (int i = 0; i < resultMap.rows; i++){
		for (int j = 0; j < resultMap.cols; j++){
			cv::Point rsPoint(j, i);
			float d = float(dImage_Meter.at<ushort>(rsPoint + orgTemplate));
			if (d != 0 && d < depthThresh){ 
				resultMap.at<float>(rsPoint) = D_BETA * ((depthThresh - d) / depthThresh);		}
			else{	
				resultMap.at<float>(rsPoint) = D_BETA * ( float(i) / resultMap.rows );	}
		}
	}
}

void compute_Normal_RespondMap(cv::Mat & normalMap, cv::Mat & resultMap)
{
	resultMap = cv::Mat(matchResult_Size, CV_32FC1, cv::Scalar(0));
	for (int i = 0; i < resultMap.rows; i++)
	{
		for (int j = 0; j < resultMap.cols; j++)
		{
			cv::Point rsPoint(j, i);
			cv::Vec3f n = normalMap.at<cv::Vec3f>(rsPoint + orgTemplate);
			if (!std::isnan(n[2]))
			{
				resultMap.at<float>(rsPoint) = D_GAMMA * fabs(n.dot(cv::Vec3f(0, 1, 0)));
			}
		}
	}
}

void cpu_matchingTemplates()
{}

int main()
{
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Synthia_RGBD_HALF);
	cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << m_camInfo.fx, 0, m_camInfo.cx, 0, m_camInfo.fy, m_camInfo.cy, 0, 0, 1);
	cv::rgbd::RgbdNormals n_estimate(processImg_Size.height, 
		processImg_Size.width, 
		CV_32F, cameraMatrix, 5, 
		cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_SRI);

	MatchingKernel mKernel;
	generateTemplate(mKernel);
	displayKernel(mKernel);


	//cv::imshow("K", mKernel.left_grad);
	//cv::waitKey();
	
	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg/*"../Data/58-rgb.png"*/);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		if (needResize)	{
			cv::resize(img, img, processImg_Size, 0, 0, cv::INTER_NEAREST);
			cv::resize(dimg, dimg, processImg_Size, 0, 0, cv::INTER_NEAREST);
		}

		cv::cuda::GpuMat dev_dMat(dimg);
		cv::cuda::GpuMat dev_xyzMap(dimg.rows, dimg.cols, CV_32FC3);

		ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
		cv::Mat xyzMap, normalMap;
		dev_xyzMap.download(xyzMap);

		n_estimate(xyzMap, normalMap);
		cv::Mat rsDMap,rsDMap_2;
		compute_Normal_RespondMap(normalMap, rsDMap);
		compute_Geometric_Map(dimg, rsDMap_2);

		cv::Mat imgProc = img(laneRegion);
		cv::Mat imgGray, imgBin;
		cv::cvtColor(imgProc, imgGray, cv::COLOR_RGB2GRAY);
		cv::threshold(imgGray, imgBin, 100, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		//imgBin = imgGray;s
		
		cv::Mat match_right, match_left;
		cv::matchTemplate(imgBin, mKernel.right, match_right, CV_TM_CCOEFF_NORMED);
		cv::matchTemplate(imgBin, mKernel.left, match_left, CV_TM_CCOEFF_NORMED);
		
		//cv::Mat geoMat;computeGeometricMap(dimg, geoMat);

		double right_MaxVal,left_MaxVal;

#ifdef DISPLAY_OLD_METHOD
		cv::Point right_MaxLoc_old = cpu_findMinmax(match_right, right_MaxVal);
		cv::Point left_MaxLoc_old = cpu_findMinmax(match_left, left_MaxVal);
		cv::putText(img, std::to_string(right_MaxVal), toOriginal(right_MaxLoc_old) - cv::Point(20, 15),
			cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, CV_COLOR_RED, 1);
		cv::putText(img, std::to_string(left_MaxVal), toOriginal(left_MaxLoc_old) - cv::Point(20, 15),
			cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, CV_COLOR_BLUE, 1);
		cv::circle(img, toOriginal(right_MaxLoc_old), 5, CV_COLOR_GREEN, 2);
		cv::circle(img, toOriginal(left_MaxLoc_old), 5, CV_COLOR_GREEN, 2);
#endif
		cv::Point right_MaxLoc = cpu_findMinmax(match_right + rsDMap + rsDMap_2, right_MaxVal);
		cv::Point left_MaxLoc = cpu_findMinmax(match_left + rsDMap + rsDMap_2, left_MaxVal);

		cv::circle(img, toOriginal(right_MaxLoc), 7, CV_COLOR_RED, 2);
		cv::circle(img, toOriginal(left_MaxLoc), 7, CV_COLOR_BLUE, 2);


		cv::putText(img, std::to_string(count), cv::Point(500, 300), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

		cv::imshow("color", img);
		cv::imshow("gray", imgBin);
		cv::imshow("norm", rsDMap);

		cv::imshow("rs", match_left + rsDMap + rsDMap_2);
		int key = cv::waitKey();
		
		if (key == 27) break;
		if (key == 32) { count-=2; };

		if (key == 'p') cv::waitKey();
		if (key == 's') cv::imwrite("test" + std::to_string(count)+".png",img);
		
		count++;
	}
}