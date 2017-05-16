#ifndef __IMG_PROC_H
#define __IMG_PROC_H

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include "types.h"

cv::Vec4f fast_PlaneDetect(const cv::Mat & depthMat, cv::Mat & rgbMat, const ImgProc3D::Intr & camInfo, bool img320x240=false);

//CPU CODE
void convertToXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Mat & xyzMat);
void fillLaneMap2D(cv::Mat & xyzMat, cv::Mat & lane2D, cv::Vec4f planeModel, float threshold=0.05f);
void create2DGrid(cv::Mat & lane2DMap, cv::Mat & colorMap, cv::Mat & gridMap);

cv::Point3f getPointXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Point2i p);

//Template matching utilities
cv::Rect createSafeRect(cv::Point tl_point, cv::Size imgSize, cv::Size preferedRectSize);
cv::Vec2f getAnglePCA(cv::Mat & tmpRS);
void getLinePoints_SlindingBox_(cv::Mat & tmpResult,
	std::vector<cv::Point> & listPoints, 
	cv::Point initPoint, cv::Vec2f pca_rs,
	cv::Size boxSize = cv::Size(32,32), 
	float jumpStep = 32);

//HUNG CODE:
void line_equation(cv::Point p1, cv::Point p2, cv::Point3f& equa);
float cal_gradient(cv::Point3f p);
bool cmp_function(cv::Point3f a, cv::Point3f b);
int count_white(const cv::Mat& image, cv::Point topleft, int mask_size = 9);

void line_processing(const cv::Mat& image, cv::Point first, int flag, std::vector<cv::Point>& line, 
    int off_set_x, int off_set_y, int mask_size = 9);

void find_initial_point(const cv::Mat& image, cv::Point& best_p_left, cv::Point& best_p_right,
     bool& flag_left, bool& flag_right, int off_set_x, int off_set_y, int mask_size = 9);

void find_lane(const cv::Mat& image, std::vector<cv::Point>& left_line, std::vector<cv::Point>& right_line);  

std::vector<cv::Point3f> find_lane_center(const cv::Mat& image, int radius = 65, int mask_size = 9); 

cv::Point3f fit_lane(std::vector<cv::Point3f> centers, int range, int left, bool& flag);

bool intersect(cv::Point3f p1, cv::Point3f p2, cv::Point2f& intersP);

bool find_curve_point(const cv::Mat& image, cv::Point2f& intersP);

#endif