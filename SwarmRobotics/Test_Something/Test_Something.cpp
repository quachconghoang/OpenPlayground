// Test_Something.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

string testFile = "C:\\Users\\HoangQC\\Desktop\\lossMask.png";
string inPath = "C:\\Users\\HoangQC\\Desktop\\Test\\";
string inFile = "lossMask_";

inline string getInFile(int num){ return inPath + inFile + std::to_string(num) + ".png"; }
inline string getOutFile(int num){ return inPath + inFile + std::to_string(num) + "_r.png"; }

void test()
{
	for (int f = 0; f < 20;f++)
	{
		Mat src = cv::imread(getInFile(f));
		Mat src_gray;
		cv::imshow("src", src);

		//cv::resize(src, src, cv::Size(src.cols / 2, src.rows / 2), INTER_NEAREST);
		cv::cvtColor(src, src_gray, COLOR_BGR2GRAY);

		vector<Vec2f> lines;
		int64 t1 = cv::getTickCount();
		HoughLines(src_gray, lines, 1, CV_PI / 180, 100, 0, 0);
		int64 t2 = cv::getTickCount();
		double run_Time = (t2 - t1) / cv::getTickFrequency();
		std::cout << run_Time << "ms \n";

		// draw lines
		for (size_t i = 0; i < 1; i++)
		{
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			std::cout << theta * 180 / CV_PI <<endl;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(src, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
		}
		imshow("Detected lines", src);
		imwrite(getOutFile(f), src);
		cv::waitKey();
	}
}


int _tmain(int argc, _TCHAR* argv[])
{
	//test();
	//return 0;

	Mat src = cv::imread(getInFile(0));
	Mat src_gray;
	cv::imshow("src",src);
	//cv::waitKey();

	//cv::resize(src, src, cv::Size(src.cols / 2, src.rows / 2), INTER_NEAREST);
	cv::cvtColor(src, src_gray, COLOR_BGR2GRAY);

	vector<Vec2f> lines;
	int64 t1 = cv::getTickCount();
	HoughLines(src_gray, lines, 1, CV_PI / 180, 100, 0, 0);
	int64 t2 = cv::getTickCount();
	double run_Time = (t2 - t1) / cv::getTickFrequency();
	std::cout << run_Time << "ms \n";
	
	// draw lines
	for (size_t i = 0; i < 1; i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(src, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	}
	cout << lines[0][1] * 180 / CV_PI << endl;
	imshow("Detected lines", src);
	cv::waitKey();
	
	return 0;
}

