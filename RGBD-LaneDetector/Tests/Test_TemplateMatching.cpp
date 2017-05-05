#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646

std::string dirPath = "D:/LaneData/Sample_30-04/";
cv::Size templateSize(32, 32);
int count = 2100;

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);
void depthMask(const cv::Mat & depthImg, cv::Mat & mask);

//1. Template matching: 3 template

//2. Plane estimation: 3 regions

//3. Polyline fitting: projection and fitting

cv::Mat tmp_left, tmp_right;
cv::Mat tmp_left_s, tmp_right_s;

void generateTemplate()
{
	//cv::Size templateSize(32, 32);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_left_s = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right_s = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	cv::line(tmp_right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 8);
	cv::line(tmp_left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 8);
	cv::line(tmp_right_s, cv::Point(0, 0), cv::Point(48, 32), cv::Scalar(255), 4);
	cv::line(tmp_left_s, cv::Point(0, 32), cv::Point(48, 0), cv::Scalar(255), 4);

	//cv::imshow("tmp-Left", tmp_left);
	//cv::imshow("tmp-Right", tmp_right);
	//cv::waitKey();
}

void templateProcessing(cv::Mat & imgBin, const cv::Mat &tmp, cv::Mat & matchingResult, std::string id = "NONAME")
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

void fillRegion(cv::Mat & dMat, cv::Mat & rgbMat, cv::Rect search_rect)
{
	for (int i = 0; i < search_rect.height; i++)
	{
		for (int j = 0; j < search_rect.width; j++)
		{
			cv::Point c_point = search_rect.tl() + cv::Point(j, i);
			if (cv::Rect(0,0,dMat.cols,dMat.cols).contains(c_point))
			{
				ushort realDepth = dMat.at<ushort>(c_point);
				if (realDepth != 0 && realDepth < 50000)	{
					rgbMat.at<cv::Vec3b>(c_point)[2] = 255;
				}
			}
		}
	}
}

void laneLineSampling(cv::Mat & rgbimg, cv::Mat & dimg)
{
	cv::Point op(0, 80);
	cv::Rect cropRect(op.x, op.y, 640, 320);
	cv::Mat img = rgbimg(cropRect);

	cv::Mat imgGray, imgBin;
	cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
	cv::imshow("imgGray", imgGray);
	cv::threshold(imgGray, imgBin, 150, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
	cv::imshow("imgBin", imgBin);
	//cv::waitKey();

	//cv::Mat dMask = cv::Mat(dimg.rows, dimg.cols, CV_8UC1, cv::Scalar(0));
	//depthMask(dimg, dMask);

	cv::Mat result_left, result_right, result_prev;
	templateProcessing(imgBin, tmp_left, result_left);
	templateProcessing(imgBin, tmp_right, result_right);

	double maxVal_Left, maxVal_Right;
	cv::Point matchLocLeft = getBestMatchLoc(result_left, maxVal_Left) + op;
	cv::Point matchLocRight = getBestMatchLoc(result_right, maxVal_Right) + op;
	cv::Rect tmpRect_Left = cv::Rect(matchLocLeft, cv::Size(32,32));
	cv::Rect tmpRect_Right = cv::Rect(matchLocRight, cv::Size(32, 32));

	fillRegion(dimg, rgbimg, tmpRect_Left);
	fillRegion(dimg, rgbimg, tmpRect_Right);

	cv::rectangle(rgbimg, tmpRect_Left, cv::Scalar(0, 255, 0), 2);
	cv::rectangle(rgbimg, tmpRect_Right, cv::Scalar(255, 0, 0), 2);

	//cv::threshold(result_left, result_left, 0.8, 1, CV_THRESH_BINARY);
	//cv::threshold(result_right, result_right, 0.8, 1, CV_THRESH_BINARY);
	//cv::imshow("Left", result_left);
	//cv::imshow("Right", result_right);

	cv::putText(rgbimg, std::to_string(count), cv::Point(500, 420), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
	cv::putText(rgbimg, std::to_string(maxVal_Left), matchLocLeft - cv::Point(20, 5), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
	cv::putText(rgbimg, std::to_string(maxVal_Right), matchLocRight - cv::Point(20, 5), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);

	cv::imshow("dimg", dimg);
	cv::imshow("img", rgbimg);
	
	//cv::imshow("dMask", dMask);
	//cv::waitKey();
}

int main()
{
	generateTemplate();
	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Realsense_RAW);
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		laneLineSampling(img, dimg);
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
				mask.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
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