#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-01-FOG/";
int count = 150;

cv::Size templateSize(32, 32);
cv::Point templateCenter(16, 16);
cv::Size fullImg_Size(1280, 760);
cv::Size processImg_Size(640, 380);

cv::Rect laneRegion(0, 180, 640, 200);
cv::Point orgTemplate(0 + 16,180 + 16);

cv::Point toOriginal(cv::Point p)	{ return p + laneRegion.tl() + templateCenter; }
cv::Rect toOriginal(cv::Rect r)		{ return cv::Rect(r.tl() + laneRegion.tl() + templateCenter, r.size()); }

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

cv::Point cuda_findMinmax(const cv::cuda::GpuMat & matchingResult, double & maxVal)
{
	double minVal;
	cv::Point minLoc, maxLoc;
	cv::cuda::minMaxLoc(matchingResult, &minVal, &maxVal, &minLoc, &maxLoc);
	return maxLoc;
}

#define INVALID_CVPOINT2i cv::Point2i(-1, -1)

cv::Point searchPoint(const cv::Mat & dMat, cv::Point & origin, int searchRadius, cv::RNG & rng)
{
	int tryCount = 0;
	cv::Point2i ranPoint;
	do {
		ranPoint = cv::Point2i(
			rng.uniform(origin.x - searchRadius, origin.x + searchRadius),
			rng.uniform(origin.y - searchRadius, origin.y + searchRadius));
		tryCount++;
		if (tryCount == 30)
		{
			ranPoint = INVALID_CVPOINT2i;
			break;
		}
	} while (dMat.at<ushort>(ranPoint) == 0);

	return ranPoint;
}

cv::Vec4f getPlaneModel(const cv::Mat & dMat, const ImgProc3D::Intr & camInfo, cv::Point & leftPoint, 
	cv::Point & rightPointNear, cv::Point & rightPointFar)
{
	cv::RNG rng(0xFFFFFFFF);

	cv::Point p1_loc = searchPoint(dMat, rightPointNear, 5, rng);
	cv::Point p2_loc = searchPoint(dMat, rightPointFar, 5, rng);
	cv::Point p3_loc = searchPoint(dMat, leftPoint, 5, rng);
	
	if (p3_loc == INVALID_CVPOINT2i || p2_loc == INVALID_CVPOINT2i || p3_loc == INVALID_CVPOINT2i)
	{
		std::cout << "Detect plane failed \n";
	}

	cv::Vec3f p1 = getPointXYZ(dMat, camInfo, p1_loc);
	cv::Vec3f p2 = getPointXYZ(dMat, camInfo, p2_loc);
	cv::Vec3f p3 = getPointXYZ(dMat, camInfo, p3_loc);

	cv::Vec3f v1 = cv::normalize(p2 - p1);
	cv::Vec3f v2 = cv::normalize(p3 - p1);

	cv::Vec3f planeNormal = cv::normalize(v1.cross(v2));
	float d = -planeNormal[0] * p1[0] - planeNormal[1] * p1[1] - planeNormal[2] * p1[2];//d = - ax -by -cz;
	printf("Model: %f *x + %f *y + %f *z + %f = 0\n", planeNormal[0],planeNormal[1],planeNormal[2],d);
	return cv::Vec4f(planeNormal[0], planeNormal[1], planeNormal[2], d);
}

cv::Point3f projectPointToPlane(cv::Point3f p, cv::Vec4f planeModel)
{
	float d = (planeModel[0] * p.x + planeModel[1] * p.y + planeModel[2] * p.z + planeModel[3]);
	return p - cv::Point3f(d*planeModel[0], d*planeModel[1], d*planeModel[2]);
}

cv::Point2f pointInImage(cv::Point3f p, ImgProc3D::Intr & m_camInfo)
{
	float j = (p.x / p.z)*m_camInfo.fx + m_camInfo.cx ;
	float i = (p.y / p.z)*m_camInfo.fy + m_camInfo.cy;

	return cv::Point2f(j,i);
}

int main()
{
	generateTemplate();
	std::vector<SyncFrame> dataHeaders;
	readSyncFileHeader(dirPath + "associations.txt", dataHeaders);

	ImgProc3D::Intr m_camInfo = ImgProc3D::Intr(ImgProc3D::IntrMode_Synthia_RGBD_HALF);

	//ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	/*dev_xyzMap.download(xyzMap);*/
	while (count < dataHeaders.size() - 1)
	{
		cv::Mat img = cv::imread(dirPath + dataHeaders[count].rgbImg/*"../Data/58-rgb.png"*/);
		cv::Mat dimg = cv::imread(dirPath + dataHeaders[count].depthImg, CV_LOAD_IMAGE_ANYDEPTH);

		cv::resize(img, img, processImg_Size, 0, 0, cv::INTER_NEAREST);
		cv::resize(dimg, dimg, processImg_Size, 0, 0, cv::INTER_NEAREST);

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
			left_Orient = getAnglePCA(rs_Left(left_InitRect));
			right_Orient = getAnglePCA(rs_Right(right_InitRect));
		}

		std::vector<cv::Point> rightList;
		getLinePoints_SlindingBox_(rs_Right, rightList, right_MaxLoc - cv::Point(16, 16), right_Orient, cv::Size(32, 32), 32);

		cv::imshow("RS_L", rs_Left);
		cv::imshow("RS_R", rs_Right);
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

		if (right_MaxVal > 0.5 && left_MaxVal > 0.5 && rightList.size()>2)
		{
			cv::Vec4f laneModel = getPlaneModel(dimg, m_camInfo, toOriginal(left_MaxLoc), toOriginal(rightList[0]), toOriginal(rightList.back()));
			
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
		cv::waitKey(10);
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