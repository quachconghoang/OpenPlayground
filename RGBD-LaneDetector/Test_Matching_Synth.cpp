#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

cv::Size templateSize(32, 32);
cv::Point templateCenter(16, 16);

//std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-06-FOG/";
//int count = 100;
//bool needResize = true;
//cv::Point templateCenter(16, 16);
//cv::Size fullImg_Size(1280, 760);
//cv::Size processImg_Size(640, 380);
//cv::Rect laneRegion(0, 180, 640, 200);
//cv::Point orgTemplate(0 + 16,180 + 16);

std::string dirPath = "D:/LaneData/Sample_30-04/";
int count = 2000;
bool needResize = false;
cv::Size fullImg_Size(640, 480);
cv::Size processImg_Size(640, 480);
cv::Rect laneRegion(0, 100, 640, 380);
cv::Point orgTemplate(0 + 16, 100 + 16);


const cv::Point toOriginal(cv::Point p)	{ return p + laneRegion.tl() + templateCenter; }
const cv::Rect toOriginal(cv::Rect r)		{ return cv::Rect(r.tl() + laneRegion.tl() + templateCenter, r.size()); }

cv::Size matchingResult_Size(laneRegion.width - templateSize.width + 1, 
	laneRegion.height - templateSize.height + 1);

cv::Mat tmp_left, tmp_right, tmp_center;


cv::Ptr<cv::cuda::TemplateMatching> matchingAlg;
cv::cuda::GpuMat dev_temp_left, dev_temp_right;
cv::cuda::GpuMat dev_result_left, dev_result_right;
cv::cuda::GpuMat dev_result_normalized_left, dev_result_normalized_right;

cv::cuda::GpuMat dev_xyzMap, dev_dMat;
cv::Mat xyzMap;
//cv::Mat tmp_left_s, tmp_right_s;

void generateTemplate()
{
	//cv::Size templateSize(32, 32);
	tmp_left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	tmp_right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	//tmp_center = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	cv::line(tmp_right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);
	cv::line(tmp_left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	//cv::line(tmp_center, cv::Point(16, 0), cv::Point(16, 32), cv::Scalar(255), 12);

	dev_temp_left = cv::cuda::GpuMat(templateSize, CV_8UC1);
	dev_temp_right = cv::cuda::GpuMat(templateSize, CV_8UC1);
	dev_temp_left.upload(tmp_left);
	dev_temp_right.upload(tmp_right);

	dev_result_right = cv::cuda::GpuMat(matchingResult_Size, CV_32FC1);
	dev_result_left = cv::cuda::GpuMat(matchingResult_Size, CV_32FC1);
	dev_result_normalized_right = cv::cuda::GpuMat(matchingResult_Size, CV_32FC1);
	dev_result_normalized_left = cv::cuda::GpuMat(matchingResult_Size, CV_32FC1);
	matchingAlg = cv::cuda::createTemplateMatching(CV_8UC1, CV_TM_CCOEFF_NORMED);

	dev_dMat = cv::cuda::GpuMat(processImg_Size, CV_16UC1);
	dev_xyzMap = cv::cuda::GpuMat(processImg_Size, CV_32FC3);
	xyzMap = cv::Mat(processImg_Size, CV_32FC3);
	//cv::imshow("tmp-Left", tmp_left);
	//cv::imshow("tmp-Right", tmp_right);
	//cv::imshow("tmp-Center", tmp_center);
	//cv::waitKey();
}

int main()
{
	generateTemplate();
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Realsense_RAW);

	//ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	/*dev_xyzMap.download(xyzMap);*/
	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg/*"../Data/58-rgb.png"*/);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);

		//cv::resize(img, img, processImg_Size, 0, 0, cv::INTER_NEAREST);
		//cv::resize(dimg, dimg, processImg_Size, 0, 0, cv::INTER_NEAREST);

		dev_dMat.upload(dimg);
		ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
		dev_xyzMap.download(xyzMap);

		cv::Mat imgGray, imgBin;
		cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
		//cv::imshow("imgGray", imgGray);
		cv::threshold(imgGray, imgBin, 128, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		cv::imshow("imgBin", imgBin);

		cv::Mat imgBinRoi = imgBin(laneRegion).clone();
		cv::cuda::GpuMat dev_binImg(imgBinRoi.size(), CV_8UC1);
		dev_binImg.upload(imgBinRoi);
		
		// 0. Start processing
		matchingAlg->match(dev_binImg, dev_temp_left, dev_result_left);
		matchingAlg->match(dev_binImg, dev_temp_right, dev_result_right);

		//int64 st = cv::getTickCount();
		//int64 et = cv::getTickCount();
		//printf("matching GPU = %f \n", (et - st) / cv::getTickFrequency());

		// 1. Find max points
		double left_MaxVal, right_MaxVal;
		cv::Point left_MaxLoc = cuda_findMinmax(dev_result_left, left_MaxVal);
		cv::Point right_MaxLoc = cuda_findMinmax(dev_result_right, right_MaxVal);
		
		/*if (left_MaxVal < 0.5){
			std::cout << "refine \n";
			left_MaxLoc = cuda_findMinmax(dev_result_left(cv::Rect(0, 0, dev_result_left.cols*0.6, dev_result_left.rows)), left_MaxVal);
		}*/

		// 2. PCA -|-|- angle:
		cv::cuda::normalize(dev_result_left, dev_result_normalized_left, 0, 1, cv::NORM_MINMAX, -1);
		cv::cuda::normalize(dev_result_right, dev_result_normalized_right, 0, 1, cv::NORM_MINMAX, -1);
		cv::Mat rs_Left, rs_Right;
		dev_result_normalized_left.download(rs_Left);
		dev_result_normalized_right.download(rs_Right);

		cv::Rect left_InitRect = createSafeRect(left_MaxLoc - cv::Point(16,16), rs_Left.size(), templateSize);
		cv::Rect right_InitRect = createSafeRect(right_MaxLoc - cv::Point(16, 16), rs_Right.size(), templateSize);
		
		cv::Vec2f left_Orient, right_Orient;
		if (left_MaxVal > 0.5 && right_MaxVal > 0.5) {
			cv::Mat _t_left = rs_Left(left_InitRect);
			cv::Mat _t_right = rs_Right(right_InitRect);
			left_Orient = getAnglePCA(_t_left);
			right_Orient = getAnglePCA(_t_right);
		}

		cv::imshow("RS_L", rs_Left);
		cv::imshow("RS_R", rs_Right);

		std::vector<cv::Point> rightList;
		getLinePoints_SlindingBox_(rs_Right, rightList, right_MaxLoc - cv::Point(16, 16), right_Orient, cv::Size(32, 32), 32);

		//cv::waitKey();

		cv::putText(img, std::to_string(count), cv::Point(500, 300), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		cv::circle(img, toOriginal(left_MaxLoc), 5, cv::Scalar(255, 0, 0), 2);
		cv::circle(img, toOriginal(right_MaxLoc), 5, cv::Scalar(0, 0, 255), 2);

		if (rightList.size() > 0)
		{
			cv::line(img, toOriginal(*rightList.begin()), toOriginal(rightList.back()), cv::Scalar(0, 255, 0), 2);
		}

		//cv::putText(img, std::to_string(count), cv::Point(500, 420), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		if (left_MaxVal>0.5){
			cv::arrowedLine(img, toOriginal(left_MaxLoc),
				toOriginal(left_MaxLoc - cv::Point(30 * left_Orient[0], 30 * left_Orient[1])), cv::Scalar(255, 0, 0));
			cv::putText(img, std::to_string(left_MaxVal), 
				toOriginal(left_MaxLoc) - cv::Point(20, 15),
				cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
		}
		
		if (right_MaxVal > 0.5){
			cv::rectangle(img, toOriginal(right_InitRect), cv::Scalar(0, 0, 255), 1);

			cv::arrowedLine(img, toOriginal(right_MaxLoc),
				toOriginal(right_MaxLoc - cv::Point(120 * right_Orient[0], 120 * right_Orient[1])), cv::Scalar(0, 0, 255));

			cv::putText(img, std::to_string(right_MaxVal), 
				toOriginal(right_MaxLoc) - cv::Point(20, 15),
				cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
		}

		if (right_MaxVal > 0.5 && left_MaxVal > 0.5 && rightList.size()>1)
		{
			cv::Vec4f laneModel = getPlaneModel(dimg, m_camInfo, 
				toOriginal(left_MaxLoc), 
				toOriginal(rightList[0]), 
				toOriginal(rightList.back()));
			
			for (int s = -2; s < 3; s++)
			{
				cv::Point3f sp = projectPointToPlane(cv::Point3f(s, -1.5, 0), laneModel);
				cv::Point3f ep = projectPointToPlane(cv::Point3f(s, -1.5, 9), laneModel);
				cv::Scalar color = cv::Scalar(0, 255, 0);
				if (s == -2 || s == 2) color = cv::Scalar(0, 0, 255);
				cv::line(img, pointInImage(sp, m_camInfo), pointInImage(ep, m_camInfo), color, 1);
			}

			for (int s = 1; s < 10; s++)
			{
				cv::Point3f sp = projectPointToPlane(cv::Point3f(-2, -1.5, s), laneModel);
				cv::Point3f ep = projectPointToPlane(cv::Point3f( 2, -1.5, s), laneModel);
				cv::line(img, pointInImage(sp, m_camInfo), pointInImage(ep, m_camInfo), cv::Scalar(0, 25*(10-s), 25*(s)), 1);
			}
			//cv::waitKey();
		}
		cv::imshow("color", img);
		cv::imshow("depth", dimg);
		cv::waitKey();
		count++;
	}
	

	//cv::cuda::TemplateMatching(dev_temp, dev_temp, dev_rs);

	/*cv::Mat matchingResult;
	cv::matchTemplate(tmp_right, tmp_right, matchingResult, CV_TM_CCOEFF_NORMED);
	cv::waitKey();*/
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