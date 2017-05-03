#include "opencv2/opencv.hpp"

struct ImgHeader{
	double timeStamp;
	std::string filePath;
};

struct SensorHeader{
	double timeStamp;
	cv::Vec3f accl;
};

struct PoseParams{
	double timeStamp;
	double tx, ty, tz;
	double qx, qy, qz, qw;
};

struct FullFrame
{
	ImgHeader					depthHeader;
	ImgHeader					rgbHeaders;
	std::vector<SensorHeader>	accls;
	PoseParams					imgPose;
};

struct SyncFrame
{
	std::string depthImg;
	std::string rgbImg;
};


int readSyncFileHeader(std::string fileName, std::vector<SyncFrame> & frame);