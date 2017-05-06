#ifndef __LANE_DETECTOR_H
#define __LANE_DETECTOR_H

#include "ImgProc.h"
#include "cuda/ImgProcCuda.h"

namespace ImgProc3D
{
	class LaneDetector
	{
	public:
		LaneDetector(int imgWidth, int imgHeight);
		~LaneDetector();

		void setCamera(ImgProc3D::IntrMode _mode);
		void processFrame(cv::Mat & img, cv::Mat & dimg); // DO SOMETHING
		void genarateMap2D(cv::Vec4f pmodel);
		cv::Mat laneProject2D;
		cv::Mat xyzMap,laneMap2D;
		cv::Mat currentColorMat;
		std::vector<cv::Point3f> samplePoints;

	private:
		ImgProc3D::Intr m_camInfo;
		
		//cv::Vec4f m_previous_plane_model;

		cv::Size templateSize;
		cv::Mat tmp_left, tmp_right;
		void generateTemplate();

		cv::cuda::GpuMat dev_dMat, dev_rgbMat, dev_xyzMap, dev_laneMap2D;
		void fillLaneMap();
		void detectLaneCenter();
	};
}


#endif