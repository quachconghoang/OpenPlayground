#include "opencv2/opencv.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/DataIO.h"

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646

std::string dirPath = "D:/LaneData/Sample_30-04/";
cv::Size templateSize(32, 32);
int count = 2100;
int jumpStep = 30;

void colorizeDepthImage(const cv::Mat depthImg, cv::Mat & resultMat);
void depthMask(const cv::Mat & depthImg, cv::Mat & mask);
cv::Rect safeCreateRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedSize);
cv::Vec2f pcaAnalyzing(cv::Mat & tmpRS);
void getLinePoints_SlindingBox(cv::Mat & tmpResult, std::vector<cv::Point> & listPoints, cv::Vec2f pca_rs,
	cv::Point initPoint, cv::Size boxSize = cv::Size(32,32), float jumpStep = 32);

//1. Template matching: 3 template

//2. Plane estimation: 3 regions

//3. Polyline fitting: projection and fitting

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
	//cv::normalize(matchingResult, matchingResult, 0, 1, cv::NORM_MINMAX, -1);
	//cv::threshold(matchingResult, matchingResult, 0.7, 1, CV_THRESH_BINARY);
	cv::imshow(id, matchingResult);
}

cv::Point getBestMatchLoc(const cv::Mat & matchingResult, double & maxVal, cv::Vec2f & orientVal, std::string id = "Matching")
{
	double minVal;
	cv::Point minLoc, maxLoc;
	cv::minMaxLoc(matchingResult, &minVal, &maxVal, &minLoc, &maxLoc);

	cv::Rect safeRect = safeCreateRect(maxLoc - cv::Point(16, 16), matchingResult.size(), templateSize);
	cv::Mat tmp = matchingResult(safeRect);
	//cv::normalize(tmp, tmp, 0, 1, cv::NORM_MINMAX, -1);
	//cv::threshold(tmp, tmp, 0.7, 1, CV_THRESH_BINARY);
	cv::imshow(id, tmp);
	orientVal = pcaAnalyzing(tmp);
	return maxLoc;
}

cv::Vec2f pcaAnalyzing(cv::Mat & tmpRS)
{
	cv::normalize(tmpRS, tmpRS, 0, 1, cv::NORM_MINMAX, -1);
	std::vector<cv::Point2d> pcaData;
	for (int i = 0; i < tmpRS.rows; i++)
	{
		for (int j = 0; j < tmpRS.cols; j++)
		{
			if (tmpRS.at<float>(i,j) > 0.8f)
			{
				pcaData.push_back(cv::Point2d(j, i));
			}
		}
	}

	cv::Mat data_pts = cv::Mat(pcaData.size(), 2, CV_64FC1, &pcaData[0]);
	//std::cout << data_pts;

	//Perform PCA analysis
	cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
	//Store the center of the object
	cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
		static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
	//Store the eigenvalues and eigenvectors
	std::vector<cv::Point2d> eigen_vecs(2);
	std::vector<double> eigen_val(2);
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
	}
	//std::cout << eigen_vecs << " \n " << eigen_val[0] << "-" << eigen_val[1] <<"\n == \n";
	return cv::Vec2f(eigen_vecs[0].x,eigen_vecs[0].y);
}

int countNonZeroCenter(cv::Mat & _map, cv::Point & center)
{
	center = cv::Point(0, 0);
	//cv::imshow("MapTMP", _map);
	//cv::waitKey();
	int count = 0;
	for (int i = 0; i < _map.rows; i++)
	{
		for (int j = 0; j < _map.cols; j++)
		{
			if (_map.at<float>(i,j) > 0.1)
			{
				count++;
				center += cv::Point(j, i);
			}
		}
	}
	if (count!=0)
	{
		center /= count;
	}
	return count;
}

void getLinePoints_SlindingBox(cv::Mat & tmpResult, std::vector<cv::Point> & listPoints, cv::Point initPoint, cv::Vec2f pca_rs,
	cv::Size boxSize /* = cv::Size(32,32) */, float jumpStep /* = 32 */)
{
	cv::normalize(tmpResult, tmpResult, 0, 1, cv::NORM_MINMAX, -1);
	cv::threshold(tmpResult, tmpResult, 0.2, 1, CV_THRESH_BINARY);

	cv::Rect rsRect = cv::Rect(0, 0, tmpResult.cols, tmpResult.rows);
	cv::Point jumpDistance(jumpStep * fabs(pca_rs[0]), jumpStep * fabs(pca_rs[1]));

	//Go Down --||--
	cv::Point boxTL = initPoint;
	cv::Point boxBR = initPoint + cv::Point(jumpStep, jumpStep);
	while (rsRect.contains(boxBR) && rsRect.contains(boxTL))
	{
		cv::Rect newBox(boxTL, boxBR);
		cv::Point tmpCenter;
		int nonZero = countNonZeroCenter(tmpResult(newBox), tmpCenter);
		if (nonZero < 20) {break;}
		else{
			listPoints.push_back(tmpCenter+boxTL);
			boxTL += tmpCenter;
			boxBR = boxTL + jumpDistance;
		}
	}

	//Go Up --||--
	boxTL = initPoint - jumpDistance;
	boxBR = initPoint;
	while (rsRect.contains(boxBR) && rsRect.contains(boxTL))
	{
		cv::Rect newBox(boxTL, boxBR); 
		cv::Point tmpCenter;
		int nonZero = countNonZeroCenter(tmpResult(newBox), tmpCenter);
		if (nonZero < 20){
			break;
		}
		else{
			listPoints.push_back(tmpCenter + boxTL);
			boxTL -= tmpCenter;
			boxBR = boxTL + jumpDistance;
		}
	}
		
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


int preferedThresh = 128;
void laneLineSampling(cv::Mat & rgbimg, cv::Mat & dimg)
{
	cv::Point op(0, 80);
	cv::Rect cropRect(op.x, op.y, 640, 320);
	cv::Mat img = rgbimg(cropRect);

	cv::Mat imgGray, imgBin;
	cv::cvtColor(img, imgGray, cv::COLOR_RGB2GRAY);
	cv::imshow("imgGray", imgGray);
	cv::threshold(imgGray, imgBin, preferedThresh, 255, CV_THRESH_TOZERO); //CV_THRESH_OTSU //CV_THRESH_TOZERO
	cv::imshow("imgBin", imgBin);
	//cv::waitKey();

	//cv::Mat dMask = cv::Mat(dimg.rows, dimg.cols, CV_8UC1, cv::Scalar(0));
	//depthMask(dimg, dMask);

	cv::Mat result_left, result_right, result_right_enhance;
	templateProcessing(imgBin, tmp_left, result_left,"RS_L");
	templateProcessing(imgBin, tmp_right, result_right, "RS_R");

	double maxVal_Left, maxVal_Right;
	cv::Vec2f orient_Left, orient_Right;
	cv::Point matchLocLeft = getBestMatchLoc(result_left, maxVal_Left, orient_Left, "Left_TMP") ;
	cv::Point matchLocRight = getBestMatchLoc(result_right, maxVal_Right, orient_Right, "Right_TMP");
	
	tmp_right_s = cv::Mat(templateSize, CV_8UC1, cv::Scalar(0));
	if (maxVal_Right > 0.5)
	{
		tmp_right = tmp_right_s;
		//Adjust Thresh
		/*cv::Point center_tmp = matchLocRight + cv::Point(16, 16);
		int color = imgGray.at<uchar>(center_tmp)+
			imgGray.at<uchar>(center_tmp - cv::Point(2, 0)) +
			imgGray.at<uchar>(center_tmp - cv::Point(0, 2)) +
			imgGray.at<uchar>(center_tmp - cv::Point(-2, 0)) +
			imgGray.at<uchar>(center_tmp - cv::Point(0, -2));
		preferedThresh = color / 5-10*/;
		cv::Mat curr_gray = imgGray(safeCreateRect(matchLocRight + cv::Point(16, 16), cv::Size(imgGray.cols, imgGray.rows), cv::Size(32, 32)));
		cv::imshow("Gray", curr_gray);
	}
	else
	{
		tmp_right = tmp_right_def;
		preferedThresh = 128;
	}
	cv::line(tmp_right_s, cv::Point(0, 0), cv::Point(60 * fabs(orient_Right[0]), 60 * fabs(orient_Right[1])), cv::Scalar(255), 4);
	cv::imshow("tmp _RHS", tmp_right_s);
	templateProcessing(imgBin, tmp_right_s, result_right_enhance, "RS_R_Enhance");

	std::vector<cv::Point> rightList;
	getLinePoints_SlindingBox(result_right_enhance, rightList, matchLocRight - cv::Point(16, 16), orient_Right, cv::Size(32, 32), 32);
	//fillRegion(dimg, rgbimg, tmpRect_Left);
	//fillRegion(dimg, rgbimg, tmpRect_Right);
	matchLocLeft += (op);
	matchLocRight += (op - cv::Point(16, 16));
	cv::Rect tmpRect_Left = cv::Rect(matchLocLeft, cv::Size(32, 32));
	cv::Rect tmpRect_Right = cv::Rect(matchLocRight, cv::Size(32, 32));
	cv::rectangle(rgbimg, tmpRect_Left, cv::Scalar(0, 255, 0), 2);
	cv::rectangle(rgbimg, tmpRect_Right, cv::Scalar(255, 0, 0), 2);

	//cv::imshow("Left-template", result_left);
	//cv::imshow("Right-template",result_right);

	//cv::threshold(result_left, result_left, 0.8, 1, CV_THRESH_BINARY);
	//cv::threshold(result_right, result_right, 0.8, 1, CV_THRESH_BINARY);
	//cv::imshow("Left", result_left);
	//cv::imshow("Right", result_right);

	int colorGray = 0;
	for (int i = 0; i < rightList.size(); i++)
	{
		cv::circle(rgbimg, rightList[i] + op, 3, cv::Scalar(0,0,255),2);
		colorGray += imgGray.at<uchar>(rightList[i]);
	}
	if (rightList.size()!=0)
	{
		preferedThresh = colorGray / rightList.size() - 5;
	}
	else
	{
		preferedThresh = 128;
	}

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

cv::Rect safeCreateRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedSize)
{
	int safeWidth = preferedSize.width;
	int safeHeight = preferedSize.height;

	if (tl_point.x < 0) tl_point.x = 0;
	if (tl_point.y < 0) tl_point.y = 0;

	if (safeWidth + tl_point.x > imgSize.width)
		safeWidth = imgSize.width - tl_point.x;
	if (safeHeight + tl_point.y > imgSize.height)
		safeHeight = imgSize.height - tl_point.y;


	return cv::Rect(tl_point, cv::Size(safeWidth, safeHeight));
}