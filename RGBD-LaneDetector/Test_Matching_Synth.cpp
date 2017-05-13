#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-SUMMER/";
int count = 0;
cv::Size templateSize(32, 32);

cv::Mat tmp_left, tmp_right, tmp_right_def;
cv::Mat tmp_left_s, tmp_right_s;

void generateTemplate()
{
	//cv::Size templateSize(32, 32);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_left_s = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right_s = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	tmp_right_def = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	cv::line(tmp_right_def, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 8);

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
	//matchingResult.create(result_rows, result_cols, CV_32FC1);
	cv::matchTemplate(imgBin, tmp, matchingResult, match_method);

	//cv::GaussianBlur(matchingResult, matchingResult, cv::Size(5, 5), 0, 0);
	cv::normalize(matchingResult, matchingResult, 0, 1, cv::NORM_MINMAX, -1);
	cv::threshold(matchingResult, matchingResult, 0.95, 1, CV_THRESH_BINARY);
	cv::imshow(id, matchingResult);
}

int main()
{
	generateTemplate();

	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);
		cv::imshow("dimg", dimg);
		cv::imshow("img", img);


		cv::Mat imgGray, imgBin;
		cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
		cv::imshow("imgGray", imgGray);
		cv::threshold(imgGray, imgBin, 150, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		cv::imshow("imgBin", imgBin);

		cv::Mat result_left, result_right, result_right_enhance;
		templateProcessing(imgBin, tmp_right, result_right, "RS_R");

		cv::waitKey(10);
		count++;
	}
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