#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

int64 st, et;

#define LOG_START st=cv::getTickCount();
#define LOG_STOP printf("time = %f \n", (cv::getTickCount()-st)/cv::getTickFrequency());
#define CV_COLOR_RED cv::Scalar(0, 0, 255)
#define CV_COLOR_GREEN cv::Scalar(0, 255, 0)
#define CV_COLOR_BLUE cv::Scalar(255, 0, 0)

#define D_BETA 0.2f


using namespace cv;

cv::Size templateSize(32, 32);
cv::Point templateCenter(16, 16);

 //Synthetic data params
//std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-SUMMER/";
//int count = 200;
//bool needResize = true;
//cv::Size fullImg_Size(1280, 760);
//cv::Size processImg_Size(640, 380);
//cv::Rect laneRegion(0, 180, 640, 200);
//cv::Point orgTemplate(0 + 16, 180 + 16);

// Real data params
std::string dirPath = "D:/LaneData/Sample_30-04/";
int count = 1800;
bool needResize = false;
cv::Size fullImg_Size(640, 480);
cv::Size processImg_Size(640, 480);
cv::Rect laneRegion(0, 100, 640, 380);
cv::Point orgTemplate(0 + 16, 100 + 16);

cv::Point toOriginal(cv::Point p)	{ return p + laneRegion.tl() + templateCenter; }
cv::Rect toOriginal(cv::Rect r)		{ return cv::Rect(r.tl() + laneRegion.tl() + templateCenter, r.size()); }
cv::Size matchResult_Size(laneRegion.width - templateSize.width + 1,
	laneRegion.height - templateSize.height + 1);

void computeGeometricMap(cv::Mat & dImage_Meter, cv::Mat & resultMap)
{
	resultMap = cv::Mat(matchResult_Size, CV_32FC1, cv::Scalar(0));
	for (int i = 0; i < resultMap.rows; i++)
	{
		for (int j = 0; j < resultMap.cols; j++)
		{
			cv::Point rsPoint(j, i);
			cv::Point dPoint = rsPoint + orgTemplate;

			float d = dImage_Meter.at<float>(dPoint);
			if (d!=0 && d < 10){
				resultMap.at<float>(rsPoint) = D_BETA*((10-d) / 10);
			}
			else{
				resultMap.at<float>(rsPoint) = 0.5f * D_BETA*( float(i) / resultMap.rows);
			}
		}
	}
}

void normalGrid(cv::Mat & xyzMat)
{
	cv::Size _grid;
}

struct MatchingKernel
{
	cv::Mat left, right, center;
};

void generateTemplate(MatchingKernel & kernel)
{
	kernel.left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	kernel.right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	//kernel.center = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	
	cv::line(kernel.left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	cv::line(kernel.right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);
	//cv::line(kernel.center, cv::Point(16, 0), cv::Point(16, 32), cv::Scalar(255), 10);

}

void displayKernel(MatchingKernel & kernel)
{
	cv::Mat displayMat(88,88,CV_8UC1,128);
	kernel.left.copyTo(displayMat(cv::Rect(8, 8, 32, 32)));
	kernel.right.copyTo(displayMat(cv::Rect(48, 8, 32, 32)));
	cv::imshow("K", displayMat);
}

int main()
{
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Realsense_RAW);

	MatchingKernel mKernel;
	generateTemplate(mKernel);
	displayKernel(mKernel);
	//cv::imshow("K", mKernel.left_grad);
	cv::waitKey();
	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg/*"../Data/58-rgb.png"*/);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		if (needResize)	{
			cv::resize(img, img, processImg_Size, 0, 0, cv::INTER_NEAREST);
			cv::resize(dimg, dimg, processImg_Size, 0, 0, cv::INTER_NEAREST);
		}

		cv::Mat dImgMeter, depthRSMap;
		dimg.convertTo(dImgMeter, CV_32FC1);
		dImgMeter /= m_camInfo.scale;

		computeGeometricMap(dImgMeter, depthRSMap);

		cv::Mat imgGray, imgBin;
		cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
		cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		imgBin = imgBin(laneRegion);
		
		cv::Mat match_right, match_left;
		cv::matchTemplate(imgBin, mKernel.right, match_right, CV_TM_CCOEFF_NORMED);

		
		double right_MaxVal;
		cv::Point right_MaxLoc = cpu_findMinmax(match_right, right_MaxVal);
		match_right += depthRSMap;
		cv::Point test_MaxLoc = cpu_findMinmax(match_right, right_MaxVal);

		cv::circle(img, toOriginal(right_MaxLoc), 7, CV_COLOR_RED, 2);
		cv::circle(img, toOriginal(test_MaxLoc), 5, CV_COLOR_GREEN, 2);
		cv::putText(img, std::to_string(count), cv::Point(500, 300), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

		cv::imshow("color", img);
		cv::imshow("depth", dimg*3);
		cv::imshow("BIN", imgBin);

		depthRSMap.convertTo(depthRSMap, CV_8U, -255.0 / 0.2, 255.0);
		cv::applyColorMap(depthRSMap, depthRSMap, cv::COLORMAP_OCEAN);
		cv::imshow("rsX", depthRSMap);
		cv::imshow("rs", match_right);
		char key = cv::waitKey();
		if (key == 27) break;
		if (key == 'p') cv::waitKey();
		count++;
	}
}