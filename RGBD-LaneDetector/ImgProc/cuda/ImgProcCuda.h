#ifndef __IMG_PROC_CUDA_H
#define __IMG_PROC_CUDA_H

#include <opencv2/core.hpp>

#include <opencv2/core/cuda.hpp>
#include "../types.h"

//2D Map 512x512 - 1 pixels = 2cm
#define LANE_MAP_SIZE 512
#define LANE_MAP_SCALE 50
#define OBJ_DISTANCE_THRESHOLD 1.5f

namespace ImgProc3D
{
	void convertTo_Point3fMap(cv::cuda::GpuMat & depthMat, const ImgProc3D::Intr camInfo, cv::cuda::GpuMat & xyzMat);
	void convertTo_NormalsMap(cv::cuda::GpuMat & xyzMat, cv::cuda::GpuMat & normalMap);
	void genPlane2DMap(cv::cuda::GpuMat & xyzMat, cv::cuda::GpuMat & rgbMat, cv::Vec4f planeModel, cv::cuda::GpuMat & laneMap, cv::cuda::GpuMat & objMask);
}

#endif