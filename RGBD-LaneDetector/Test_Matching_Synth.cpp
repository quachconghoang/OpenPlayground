#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-SUMMER/";
int count = 150;
cv::Size templateSize(32, 32);
cv::Size processImg_Size(640, 380);
cv::Rect matchingImg_Rect(0, 180, 640, 200);

cv::Mat tmp_left, tmp_right, tmp_center;
//cv::cuda::GpuMat dev_temp_left, dev_temp_right;
//cv::Mat tmp_left_s, tmp_right_s;

void generateTemplate()
{
	//cv::Size templateSize(32, 32);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_center = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	cv::line(tmp_right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);
	cv::line(tmp_left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	cv::line(tmp_center, cv::Point(16, 0), cv::Point(16, 32), cv::Scalar(255), 12);

	//cv::imshow("tmp-Left", tmp_left);
	//cv::imshow("tmp-Right", tmp_right);
	//cv::imshow("tmp-Center", tmp_center);
	//cv::waitKey();
}

void templateProcessing(cv::Mat & imgBin, const cv::Mat &tmp, cv::Mat & matchingResult, std::string id = "NONAME")
{
	int match_method = CV_TM_CCOEFF_NORMED;
	int result_cols = imgBin.cols - tmp.cols + 1;
	int result_rows = imgBin.rows - tmp.rows + 1;
	//matchingResult.create(result_rows, result_cols, CV_32FC1);
	cv::matchTemplate(imgBin, tmp, matchingResult, match_method);

	//cv::GaussianBlur(matchingResult, matchingResult, cv::Size(5, 5), 0, 0);
	cv::normalize(matchingResult, matchingResult, 0, 1, cv::NORM_MINMAX, -1);
	cv::threshold(matchingResult, matchingResult, 0.9, 1, CV_THRESH_BINARY);
	cv::imshow(id, matchingResult);
}

int main()
{
	generateTemplate();
	cv::Mat img = cv::imread("../Data/58-rgb.png");
	cv::resize(img, img, cv::Size(640, 380), 0, 0, cv::INTER_NEAREST);
	cv::imshow("color",img);
	cv::Mat imgGray, imgBin;
	cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
	cv::imshow("imgGray", imgGray);
	cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
	cv::imshow("imgBin", imgBin);
	//cv::waitKey();

	cv::cuda::GpuMat dev_temp_left(templateSize, CV_8UC1);
	cv::cuda::GpuMat dev_temp_right(templateSize, CV_8UC1);
	cv::cuda::GpuMat dev_binImg(imgBin.size(), CV_8UC1);
	cv::cuda::GpuMat dev_rs_left, dev_rs_right;
	dev_temp_left.upload(tmp_left);
	dev_temp_right.upload(tmp_right);
	dev_binImg.upload(imgBin);

	cv::Ptr<cv::cuda::TemplateMatching> alg = cv::cuda::createTemplateMatching(CV_8UC1, CV_TM_CCOEFF_NORMED);
	alg->match(dev_binImg, dev_temp_left, dev_rs_left);
	alg->match(dev_binImg, dev_temp_right, dev_rs_right);
	cv::cuda::normalize(dev_rs_left, dev_rs_left, 0, 1, cv::NORM_MINMAX, -1);
	cv::cuda::normalize(dev_rs_right, dev_rs_right, 0, 1, cv::NORM_MINMAX, -1);

	cv::Mat dst;
	dev_rs_left.download(dst);
	cv::imshow("RS", dst);
	dev_rs_right.download(dst);
	cv::imshow("RS_R", dst);
	cv::waitKey();
	//cv::cuda::TemplateMatching(dev_temp, dev_temp, dev_rs);

	/*cv::Mat matchingResult;
	cv::matchTemplate(tmp_right, tmp_right, matchingResult, CV_TM_CCOEFF_NORMED);
	cv::waitKey();*/
	return 0;
}

int maingg()
{
	generateTemplate();

	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		cv::resize(img, img, cv::Size(640, 380), 0, 0, cv::INTER_NEAREST);
		cv::resize(dimg, dimg, cv::Size(640, 380), 0, 0, cv::INTER_NEAREST);

		cv::Mat imgGray, imgBin;
		cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
		cv::imshow("imgGray", imgGray);
		cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		cv::imshow("imgBin", imgBin);

		cv::Mat result_left, result_right, result_center;
		templateProcessing(imgBin, tmp_right, result_right, "RS_R");
		templateProcessing(imgBin, tmp_left, result_left, "RS_L");
		templateProcessing(imgBin, tmp_center, result_center, "RS_C");

		for (int i = 0; i < result_left.rows; i++)
		{
			for (int j = 0; j < result_left.cols; j++)
			{
				if (result_left.at<float>(i, j) == 1 && dimg.at<ushort>(16+i,16+j) < 2000)
				{
					img.at<cv::Vec3b>(16 + i, 16 + j) = cv::Vec3b(0,0,255);
				}
				if (result_right.at<float>(i, j) == 1 && dimg.at<ushort>(16 + i, 16 + j) < 2000)
				{
					img.at<cv::Vec3b>(16 + i, 16 + j) = cv::Vec3b(255, 0, 0);
				}

				if (result_center.at<float>(i, j) == 1 && dimg.at<ushort>(16 + i, 16 + j) < 2000)
				{
					img.at<cv::Vec3b>(16 + i, 16 + j) = cv::Vec3b(0, 255, 0);
				}
			}
		}

		cv::imshow("dimg", dimg);
		cv::imshow("img", img);

		cv::waitKey(10);
		count++;
	}

	return 0;
}

//void createHeaderFile()
//{
	//std::ofstream recordFile;
	//recordFile.open(dirPath + "associations.txt");
	//recordFile << "# RGBD recorder TUM-format (Scale = 1x centimeter = 100 ) \n";
	//recordFile << "# Date: " << std::endl;
	//recordFile << "# Intr: " << std::endl;
	//for (int i = 0; i < 944; i++)
	//{
	//	int indexData = i;
	//	char buffer[256]; sprintf(buffer, "%06d", i);
	//	std::string depFile = "Depth/" + std::string(buffer) + ".png";
	//	std::string rgbFile = "RGB/" + std::string(buffer) + ".png";
	//	recordFile << std::to_string(indexData) << " " << depFile << " " << std::to_string(indexData) << " " << rgbFile << "\n";
	//}
	//recordFile.close();
//}