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
std::string dirPath = "D:/LaneData/SynthDataLane/SEQS-06-FOG/";
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

#define TEMP_LINE_WIDTH 10
#define D_BETA 0.1f
#define D_GAMMA 0.2f
#define depthThresh 1000
#define GRAY_THRESH 100
bool use_normal = true; // Using normal constraint for all lines
bool use_depth = true; // Using depth constraint for SOLID line - not DASH lines
bool use_refined_template = false;

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

	cv::line(kernel.left, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), TEMP_LINE_WIDTH);
	cv::line(kernel.right, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), TEMP_LINE_WIDTH);
	cv::line(kernel.left_enhance, cv::Point(0, 32), cv::Point(32, 0), cv::Scalar(255), TEMP_LINE_WIDTH);
	cv::line(kernel.right_enhance, cv::Point(0, 0), cv::Point(32, 32), cv::Scalar(255), TEMP_LINE_WIDTH);
}

void updateTemplate(PCA_Result leftPCA, PCA_Result rightPCA, MatchingKernel & kernel)
{
	kernel.left_enhance.setTo(cv::Scalar(0));
	kernel.right_enhance.setTo(cv::Scalar(0));

	cv::Point l_start = templateCenter - cv::Point(30 * leftPCA._vec[0], 30 * leftPCA._vec[1]);
	cv::Point l_end = templateCenter + cv::Point(30 * leftPCA._vec[0], 30 * leftPCA._vec[1]);
	cv::line(kernel.left_enhance, l_start, l_end, cv::Scalar(255), TEMP_LINE_WIDTH);
	
	cv::Point r_start = templateCenter - cv::Point(30 * rightPCA._vec[0], 30 * rightPCA._vec[1]);
	cv::Point r_end = templateCenter + cv::Point(30 * rightPCA._vec[0], 30 * rightPCA._vec[1]);
	cv::line(kernel.right_enhance, r_start, r_end, cv::Scalar(255), TEMP_LINE_WIDTH);
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

void displayMatchingResults(cv::Mat & img, MatchingResult mRS, PCA_Result pcaRS, cv::Scalar colorCode)
{
	cv::circle(img, toOriginal(mRS.RGBD_max_loc), 7, colorCode, 2);
	cv::putText(img, std::to_string(mRS.RGBD_max_val), toOriginal(mRS.RGBD_max_loc) - cv::Point(20, 15),
		cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.3, colorCode, 1);

	cv::arrowedLine(img, toOriginal(mRS.RGBD_max_loc),
		toOriginal(mRS.RGBD_max_loc - cv::Point(60 * pcaRS._vec[0], 60 * pcaRS._vec[1])), colorCode);
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

void compute_Adjust_RSMap_1(cv::Mat & rgbd_adjust, cv::Mat & nMap, float mThreshold)
{
	for (int i = 0; i < rgbd_adjust.rows; i++){
		for (int j = 0; j < rgbd_adjust.cols; j++){
			if (rgbd_adjust.at<float>(i, j) > mThreshold){
				rgbd_adjust.at<float>(i, j) += nMap.at<float>(i, j);
			}
		}
	}
}
void compute_Adjust_RSMap_2(cv::Mat & rgbd_adjust, cv::Mat & gMap, cv::Mat & nMap, float mThreshold)
{
	for (int i = 0; i < rgbd_adjust.rows; i++){
		for (int j = 0; j < rgbd_adjust.cols; j++){
			if (rgbd_adjust.at<float>(i, j) > mThreshold){
				float adjParam = (gMap.at<float>(i, j) + nMap.at<float>(i, j));
				rgbd_adjust.at<float>(i, j) += adjParam;
			}
		}
	}
}

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

		if (use_refined_template){
			cv::matchTemplate(imgBin, mKernel.right_enhance, match_RGB_right, CV_TM_CCOEFF_NORMED);
			cv::matchTemplate(imgBin, mKernel.left_enhance, match_RGB_left, CV_TM_CCOEFF_NORMED);
		}
		else{
			cv::matchTemplate(imgBin, mKernel.right, match_RGB_right, CV_TM_CCOEFF_NORMED);
			cv::matchTemplate(imgBin, mKernel.left, match_RGB_left, CV_TM_CCOEFF_NORMED);
		}
		
		MatchingResult leftRS,rightRS;

		// Check RGB matching
		leftRS.RGB_max_loc = cpu_findMinmax(match_RGB_left, leftRS.RGB_max_val);
		rightRS.RGB_max_loc = cpu_findMinmax(match_RGB_right, rightRS.RGB_max_val);
		
		//4. RGB-D matching	
		cv::Mat match_rgbd_left = match_RGB_left;
		cv::Mat match_rgbd_right = match_RGB_right;
		if (use_depth)
		{
			compute_Adjust_RSMap_1(match_RGB_left, res_normalMap, 0.5f);
			//compute_Adjust_RSMap_2(match_RGB_left, res_depthMap, res_normalMap, 0.5f);
			compute_Adjust_RSMap_2(match_RGB_right, res_depthMap, res_normalMap, 0.5f);
		}

		leftRS.RGBD_max_loc = cpu_findMinmax(match_rgbd_left, leftRS.RGBD_max_val);
		rightRS.RGBD_max_loc = cpu_findMinmax(match_rgbd_right, rightRS.RGBD_max_val);

		cv::Rect left_InitRect, right_InitRect;
		left_InitRect = createSafeRect(leftRS.RGB_max_loc - cv::Point(16, 16), match_RGB_left.size(), templateSize);
		right_InitRect = createSafeRect(rightRS.RGB_max_loc - cv::Point(16, 16), match_RGB_right.size(), templateSize);

		// 5. Update template
		PCA_Result left_pca, right_pca;
		if (leftRS.RGB_max_val > 0.5) getAnglePCA(match_RGB_left(left_InitRect).clone(), left_pca);
		if (rightRS.RGB_max_val > 0.5) getAnglePCA(match_RGB_right(right_InitRect).clone(), right_pca);
		
		if (leftRS.RGB_max_val > 0.75 && rightRS.RGB_max_val > 0.75) {
			updateTemplate(left_pca, right_pca, mKernel);
			use_refined_template = true;
		}
		else{
			use_refined_template = false;
		}

		std::vector<cv::Point> rightList;
		getLinePoints_SlindingBox_(match_rgbd_right, rightList, rightRS.RGBD_max_loc, right_pca._vec, cv::Size(32, 32), 20);

		if (rightRS.RGB_max_val > 0.5 && leftRS.RGB_max_val > 0.5 && rightList.size() > 1)
		{
			cv::Vec4f laneModel = getPlaneModel(dimg, m_camInfo, toOriginal(leftRS.RGBD_max_loc), toOriginal(rightList[0]), toOriginal(rightList.back()));

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
				cv::Point3f ep = projectPointToPlane(cv::Point3f(2, -1.5, s), laneModel);
				cv::line(img, pointInImage(sp, m_camInfo), pointInImage(ep, m_camInfo), cv::Scalar(0, 25 * (10 - s), 25 * (s)), 1);
			}
			//cv::waitKey();
		}

		if (rightList.size() > 0){
			cv::line(img, toOriginal(*rightList.begin()), toOriginal(rightList.back()), cv::Scalar(0, 255, 0), 2);
		}


		displayKernel(mKernel);
		displayMatchingResults(img, leftRS, left_pca, CV_COLOR_BLUE);
		displayMatchingResults(img, rightRS, right_pca, CV_COLOR_RED);

		/*cv::circle(img, toOriginal(leftRS.RGB_max_loc), 5, CV_COLOR_GREEN, 2);
		cv::circle(img, toOriginal(rightRS.RGB_max_loc), 5, CV_COLOR_GREEN, 2);*/


		cv::putText(img, std::to_string(count), cv::Point(500, 300), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

		cv::imshow("color", img);
		cv::imshow("gray", imgBin);
		cv::imshow("norm", res_normalMap);

		cv::imshow("rs_L", match_rgbd_left);
		cv::imshow("rs_R", match_rgbd_right);
		int key = cv::waitKey();
		
		if (key == 27) break;
		if (key == 32) { count-=2; };

		if (key == 'p') cv::waitKey();
		if (key == 's') cv::imwrite("test" + std::to_string(count)+".png",img);
		
		count++;
	}
}