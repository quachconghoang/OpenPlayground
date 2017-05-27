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
//#define REFINE_TEMPLATE

using namespace cv;

cv::Size templateSize(32, 32);
cv::Point templateCenter(16, 16);
cv::Size cellTables(640 / 32, 480 / 32); // 20 - 15;

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

#define D_BETA 0.1f
#define D_GAMMA 0.2f
#define depthThresh 1000
#define GRAY_THRESH 80
bool use_normal = true; // Using normal constraint for all lines
bool use_depth = true; // Using depth constraint for SOLID line - not DASH lines

struct MatchingKernel
{
	cv::Mat left, right;
	cv::Mat left_enhance;
	cv::Mat right_enhance;
};

struct MatchingResult
{
	double RGB_max_val;
	cv::Point RGB_max_loc;
	double RGBD_max_val;
	cv::Point RGBD_max_loc;
};

void generateTemplate(MatchingKernel & kernel)
{
	kernel.left = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	kernel.right = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	kernel.left_enhance = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	kernel.right_enhance = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));

	cv::line(kernel.left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	cv::line(kernel.right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);
	cv::line(kernel.left_enhance, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), 12);
	cv::line(kernel.right_enhance, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), 12);
}

void updateTemplate(PCA_Result leftPCA, PCA_Result rightPCA, MatchingKernel & kernel)
{
	kernel.left_enhance.setTo(cv::Scalar(0));
	kernel.right_enhance.setTo(cv::Scalar(0));

	cv::Point l_start = templateCenter - cv::Point(30 * fabs(leftPCA._vec[0]), -30 * fabs(leftPCA._vec[1]));
	cv::Point l_end = templateCenter + cv::Point(30 * fabs(leftPCA._vec[0]), -30 * fabs(leftPCA._vec[1]));
	cv::line(kernel.left_enhance, l_start, l_end, cv::Scalar(255), 12);
	
	cv::Point r_start = templateCenter - cv::Point(30 * fabs(rightPCA._vec[0]), 30 * fabs(rightPCA._vec[1]));
	cv::Point r_end = templateCenter + cv::Point(30 * fabs(rightPCA._vec[0]), 30 * fabs(rightPCA._vec[1]));
	cv::line(kernel.right_enhance, r_start, r_end, cv::Scalar(255), 12);
}

void displayKernel(MatchingKernel & kernel)
{
	cv::Mat displayMat(88,88,CV_8UC1,128);
	kernel.left.copyTo(displayMat(cv::Rect(8, 8, 32, 32)));
	kernel.right.copyTo(displayMat(cv::Rect(48, 8, 32, 32)));
	kernel.left_enhance.copyTo(displayMat(cv::Rect(8, 48, 32, 32)));
	kernel.right_enhance.copyTo(displayMat(cv::Rect(48, 48, 32, 32)));
	cv::imshow("K", displayMat);
}

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

		//1. Compute XYZ and Normal maps
		cv::cuda::GpuMat dev_dMat(dimg);
		cv::cuda::GpuMat dev_xyzMap(dimg.rows, dimg.cols, CV_32FC3);
		ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
		cv::Mat xyzMap, normalMap;
		dev_xyzMap.download(xyzMap);
		n_estimate(xyzMap, normalMap);

		//2. Compute Geometric response map
		cv::Mat res_normalMap,res_depthMap;
		compute_Normal_RespondMap(normalMap, res_normalMap);
		compute_Geometric_Map(dimg, res_depthMap);

		//3. Template matching rgb image
		cv::Mat imgProc = img(laneRegion);
		cv::Mat imgGray, imgBin;
		cv::cvtColor(imgProc, imgGray, cv::COLOR_RGB2GRAY);
		cv::threshold(imgGray, imgBin, GRAY_THRESH, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
		
		cv::Mat match_RGB_right, match_RGB_left;
		cv::matchTemplate(imgBin, mKernel.right, match_RGB_right, CV_TM_CCOEFF_NORMED);
		cv::matchTemplate(imgBin, mKernel.left, match_RGB_left, CV_TM_CCOEFF_NORMED);
		//double right_MaxVal,left_MaxVal;

		MatchingResult leftRS,rightRS;

#ifdef DISPLAY_OLD_METHOD
		cv::Point right_MaxLoc_old = cpu_findMinmax(match_RGB_right, right_MaxVal);
		cv::Point left_MaxLoc_old = cpu_findMinmax(match_RGB_left, left_MaxVal);
		cv::putText(img, std::to_string(right_MaxVal), toOriginal(right_MaxLoc_old) - cv::Point(20, 15),
			cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, CV_COLOR_RED, 1);
		cv::putText(img, std::to_string(left_MaxVal), toOriginal(left_MaxLoc_old) - cv::Point(20, 15),
			cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, CV_COLOR_BLUE, 1);
		cv::circle(img, toOriginal(right_MaxLoc_old), 5, CV_COLOR_GREEN, 2);
		cv::circle(img, toOriginal(left_MaxLoc_old), 5, CV_COLOR_GREEN, 2);
#endif

		cv::Mat match_rgbd_left = match_RGB_left;
		cv::Mat match_rgbd_right = match_RGB_right;
		
		if (use_depth){
			//match_rgbd_left += res_depthMap;
			match_rgbd_right += res_depthMap;
		}

		if (use_normal){
			match_rgbd_left += res_normalMap;
			match_rgbd_right += res_normalMap;
		}

		// Check RGB matching
		leftRS.RGB_max_loc = cpu_findMinmax(match_RGB_left, leftRS.RGB_max_val);
		rightRS.RGB_max_loc = cpu_findMinmax(match_RGB_right, rightRS.RGB_max_val);
		
		// Check RGB-D matching
		leftRS.RGBD_max_loc = cpu_findMinmax(match_rgbd_left, leftRS.RGBD_max_val);
		rightRS.RGBD_max_loc = cpu_findMinmax(match_rgbd_right, rightRS.RGBD_max_val);

		cv::Rect left_InitRect = createSafeRect(leftRS.RGB_max_loc - cv::Point(16, 16), match_RGB_left.size(), templateSize);
		cv::Rect right_InitRect = createSafeRect(rightRS.RGB_max_loc - cv::Point(16, 16), match_RGB_right.size(), templateSize);

		PCA_Result left_pca, right_pca;

		if (leftRS.RGB_max_val > 0.7 && rightRS.RGB_max_val > 0.7) {
			getAnglePCA(match_RGB_left(left_InitRect).clone(),left_pca);
			getAnglePCA(match_RGB_right(right_InitRect).clone(), right_pca);

			updateTemplate(left_pca, right_pca, mKernel);
		}
		displayKernel(mKernel);

		cv::circle(img, toOriginal(leftRS.RGBD_max_loc), 7, CV_COLOR_BLUE, 2);
		cv::circle(img, toOriginal(rightRS.RGBD_max_loc), 7, CV_COLOR_RED, 2);

		cv::arrowedLine(img, toOriginal(leftRS.RGBD_max_loc),
			toOriginal(leftRS.RGBD_max_loc - cv::Point(30 * left_pca._vec[0], 30 * left_pca._vec[1])), CV_COLOR_BLUE);
		cv::arrowedLine(img, toOriginal(rightRS.RGBD_max_loc),
			toOriginal(rightRS.RGBD_max_loc - cv::Point(120 * right_pca._vec[0], 120 * right_pca._vec[1])), CV_COLOR_RED);



		cv::putText(img, std::to_string(count), cv::Point(500, 300), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

		cv::imshow("color", img);
		cv::imshow("gray", imgBin);
		cv::imshow("norm", res_normalMap);

		//cv::imshow("rs", match_RGB_left + res_normalMap + res_depthMap);
		int key = cv::waitKey();
		
		if (key == 27) break;
		if (key == 32) { count-=2; };

		if (key == 'p') cv::waitKey();
		if (key == 's') cv::imwrite("test" + std::to_string(count)+".png",img);
		
		count++;
	}
}