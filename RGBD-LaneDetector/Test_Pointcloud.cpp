#include "opencv2/opencv.hpp"

struct Intrinsics{
	float fx, fy;
	float cx, cy;
	float pixelToMeter = 1000.f;

	Intrinsics(float _fx, float _fy, float _cx, float _cy){
		fx = _fx; fy = _fy; cx = _cx; cy = _cy;
	}
};

struct PointCloudXYZRGB
{
	std::vector<cv::Point3f> points;
	std::vector<cv::Vec3b> colors;
};

int convertToPointcloud(cv::Mat & rgbImg, cv::Mat & depthImg, Intrinsics cam, PointCloudXYZRGB & _cloud)
{
	_cloud.points.resize(rgbImg.rows*rgbImg.cols);
	_cloud.colors.resize(rgbImg.rows*rgbImg.cols);

	cv::Mat depthImgF;
	depthImg.convertTo(depthImgF, CV_32F); // convert the image data to float type 
	depthImgF /= cam.pixelToMeter;

	int idx = 0;
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			float d = depthImgF.at<float>(i, j);
			_cloud.colors[idx] = rgbImg.at<cv::Vec3b>(i, j);

			if (d == 0)
				_cloud.points[idx] = cv::Point3f(NAN, NAN, NAN);
			else
				_cloud.points[idx] = cv::Point3f((j - cam.cx) * d / cam.fx, (i - cam.cy) * d / cam.fy, d);idx++;
		}
	}

	return 0;
}

// Depth instr : 640 480 616.442444 616.442444 319.500000 231.408646
int main()
{
	Intrinsics intr = Intrinsics(616.442444f, 616.442444f, 319.500000f, 231.408646f);
	cv::viz::Viz3d viz("show_cloud");

	cv::Mat img = cv::imread("../Data/480-rgb.png");
	cv::Mat dimg = cv::imread("../Data/480-d.png", CV_LOAD_IMAGE_ANYDEPTH);

	PointCloudXYZRGB cloudXYZrgb;
	convertToPointcloud(img, dimg, intr, cloudXYZrgb);
	viz.showWidget("w", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors)/*, cvaff*/);
	viz.showWidget("coo-sys", cv::viz::WCoordinateSystem(5.0));
	viz.spin();
}