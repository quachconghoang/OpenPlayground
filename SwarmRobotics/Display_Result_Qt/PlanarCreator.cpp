#include "stdafx.h"
#include "PlanarCreator.h"
#include <stdio.h>
#include <cmath>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;

PointCloudPtrT helper::downSampling(PointCloudPtrT & targetCloud,float leafSize)
{
	PointCloudPtrT align (new PointCloudT);
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (targetCloud);
	sor.setLeafSize (leafSize,leafSize,leafSize);
	sor.filter (*align);
	return align;
}

NormalCloudPtrT helper::normalsEstimate(PointCloudPtrT cloud_input, int neighbors)
{
	NormalEstimation<PointT, pcl::Normal> n;
	PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
	search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
	tree->setInputCloud (cloud_input);
	n.setInputCloud (cloud_input);
	n.setSearchMethod (tree);
	n.setKSearch (neighbors);
	n.compute (*normals);
	return normals;
}

PointCloudPtrT helper::statisticalOutlierRemoval(PointCloudPtrT cloud_input, int neighbors)
{
	PointCloudPtrT result_cloud (new PointCloudT);
	std::cerr << "Outline Filtering..." << std::endl;
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud_input);
	sor.setMeanK(neighbors);
	sor.setStddevMulThresh (1);
	sor.filter (*result_cloud);
	return result_cloud;
}

PointCloudPtrT helper::projectCloud(PointCloudPtrT cloud_input,pcl::ModelCoefficients::Ptr coefficients)
{
	PointCloudPtrT cloud_projected (new PointCloudT);
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_input);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	return cloud_projected;
}

PointCloudPtrT helper::hullEsitmate(PointCloudPtrT cloud_input)
{
	PointCloudPtrT cloud_hull (new PointCloudT);
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (cloud_input);
	chull.reconstruct (*cloud_hull);

	//Checking for qHull errors
	if (cloud_hull->points.size()<3)
	{
		PointCloudPtrT cloud_hull_rotated (new PointCloudT);
		PointCloudPtrT cloud_input_rotated (new PointCloudT);
		double r_angle = M_PI_2/4;
		Eigen::Affine3d rm(Eigen::AngleAxisd(r_angle, Eigen::Vector3d::UnitX()));
		Eigen::Affine3d rm_inverted(Eigen::AngleAxisd(-r_angle, Eigen::Vector3d::UnitX()));
		pcl::transformPointCloud(*cloud_input,*cloud_input_rotated,rm);
		chull.setInputCloud (cloud_input_rotated);
		chull.reconstruct (*cloud_hull);
		pcl::transformPointCloud(*cloud_hull,*cloud_hull,rm_inverted);
	}

	if (cloud_hull->points.size()<3)
	{
		PointCloudPtrT cloud_hull_rotated (new PointCloudT);
		PointCloudPtrT cloud_input_rotated (new PointCloudT);
		double r_angle = M_PI_2/4;
		Eigen::Affine3d rm(Eigen::AngleAxisd(r_angle, Eigen::Vector3d::UnitY()));
		Eigen::Affine3d rm_inverted(Eigen::AngleAxisd(-r_angle, Eigen::Vector3d::UnitY()));
		pcl::transformPointCloud(*cloud_input,*cloud_input_rotated,rm);
		chull.setInputCloud (cloud_input_rotated);
		chull.reconstruct (*cloud_hull);
		pcl::transformPointCloud(*cloud_hull,*cloud_hull,rm_inverted);
	}

	if (cloud_hull->points.size()<3)
	{
		PointCloudPtrT cloud_hull_rotated (new PointCloudT);
		PointCloudPtrT cloud_input_rotated (new PointCloudT);
		double r_angle = M_PI_2/4;
		Eigen::Affine3d rm(Eigen::AngleAxisd(r_angle, Eigen::Vector3d::UnitZ()));
		Eigen::Affine3d rm_inverted(Eigen::AngleAxisd(-r_angle, Eigen::Vector3d::UnitZ()));
		pcl::transformPointCloud(*cloud_input,*cloud_input_rotated,rm);
		chull.setInputCloud (cloud_input_rotated);
		chull.reconstruct (*cloud_hull);
		pcl::transformPointCloud(*cloud_hull,*cloud_hull,rm_inverted);
	}
	return cloud_hull;
}

double helper::areaEstimate(PointCloudPtrT cloud_input, double resolution)
{
	double resolution_x = resolution;
	int numberPoint = cloud_input->size();
	if (numberPoint<3)	return 0;
	
	double size_in_mm2 = 0;

	PointCloudPtrT cloud_filtered = downSampling(cloud_input,resolution_x);

	double multiple_count=1;
	//Down sampling until lost 25% points
	while (cloud_filtered->size() > 0.75*numberPoint)
	{
		resolution_x = resolution*(1+0.1*multiple_count);
		cloud_filtered = downSampling(cloud_input,resolution_x);
		multiple_count++;
	}

	//Calculate area
	size_in_mm2 = cloud_filtered->size()*resolution_x*resolution_x;
	return size_in_mm2/1000000;
}

pcl::PolygonMesh::Ptr helper::triangulation(PointCloudPtrT cloud_input)
{
	PointCloudPtrT cloud(cloud_input);
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	// check error and return empty polygon mesh
	if(cloud->size()< 5) return triangles;
	// Normal estimation*
	pcl::PointCloud<pcl::Normal>::Ptr normals = helper::normalsEstimate(cloud,8);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (1000);
	gp3.setMu (800);
	gp3.setMaximumNearestNeighbors (10);
	gp3.setMaximumSurfaceAngle(M_PI_2+0.1);
	gp3.setMinimumAngle(M_PI_2-0.1);
	gp3.setMaximumAngle(M_PI_2);
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*triangles);
	return triangles;
}

pcl::ModelCoefficients::Ptr estimateBestNorm(Eigen::Vector3f norm_model, Eigen::Vector3i & point_index)
{
	float normal[3];
	normal[0] = abs(norm_model.dot(Eigen::Vector3f(0,0,1)));
	normal[1] = abs(norm_model.dot(Eigen::Vector3f(0,1,0)));
	normal[2] = abs(norm_model.dot(Eigen::Vector3f(1,0,0)));
	int pIndex = std::distance(normal, std::max_element(normal, normal + 3));
	assert(pIndex<3);

	pcl::ModelCoefficients::Ptr prjCoff(new pcl::ModelCoefficients ());
	prjCoff->values.assign(4,0);
	switch (pIndex)
	{
	default:
		prjCoff->values[2]=1; point_index = Eigen::Vector3i(0,1,2); break;
	case 1:
		prjCoff->values[1]=1; point_index = Eigen::Vector3i(0,2,1); break;
	case 2:
		prjCoff->values[0]=1; point_index = Eigen::Vector3i(1,2,0); break;
	}
	return prjCoff;
}

std::vector<cv::Point2i> convertPolygon3Dto2D(const PointCloudPtrT boundary, const pcl::ModelCoefficients::Ptr prjCoff)
{
	std::vector<cv::Point2i> polygon;
	PointCloudPtrT boundary_projected = helper::projectCloud(boundary,prjCoff);
	TYPE_PROJECTED_PLANE type_plane = TYPE_PROJECTED_PLANE_001;
	if (prjCoff->values[0]>0) type_plane = TYPE_PROJECTED_PLANE_100;
	if (prjCoff->values[1]>0) type_plane = TYPE_PROJECTED_PLANE_010;

	for (int i=0; i<boundary_projected->size(); i++)
	{
		PointT p = boundary_projected->points[i];
		cv::Point2i p2d;
		switch (type_plane)
		{
		case TYPE_PROJECTED_PLANE_001:
			p2d =  cv::Point2i(p.x, p.y);
			break;
		case TYPE_PROJECTED_PLANE_010:
			p2d =  cv::Point2i(p.x, p.z);
			break;
		case TYPE_PROJECTED_PLANE_100:
			p2d =  cv::Point2i(p.y, p.z);
			break;
		default:
			break;
		}
		polygon.push_back(p2d);
	}
	return polygon;
}

bool checkBlocking(cv::Point2f p , std::vector<std::vector<cv::Point2i>> & blockPolygons)
{
	for (int it=0;it<blockPolygons.size();it++)
	{
		if (cv::pointPolygonTest( blockPolygons[it], p, false ) == 1)
		{
			return true;
		}
	}
	return false;
}

PointCloudPtrT helper::generateDisplayCloud(const pcl::ModelCoefficients::Ptr coefficients, const PointCloudPtrT boundary, const std::vector<PointCloudT> & blockBoundary)
{
	PointCloudPtrT boundary_projected(new PointCloudT);
	PointCloudPtrT result_cloud(new PointCloudT);

	//check if too little to generate polygon
	if (boundary->size()<3) return result_cloud;
	
	Eigen::Vector3i pIdx(0,1,2);//Index of X-Y-Z

	Eigen::Vector3f norm_model(coefficients->values[0],coefficients->values[1],coefficients->values[2]);

	pcl::ModelCoefficients::Ptr prjCoff = estimateBestNorm(norm_model,pIdx);

	std::vector<cv::Point2i> polygon = convertPolygon3Dto2D(boundary,prjCoff);


	std::vector<std::vector<cv::Point2i>> blockPolygons;
	for (int i=0; i<blockBoundary.size(); i++)
	{
		std::vector<cv::Point2i> plg = convertPolygon3Dto2D(blockBoundary[i].makeShared(),prjCoff);
		blockPolygons.push_back(plg);
		
	}

	//SWAP a-b-c-d to new x-y-z
	float a = coefficients->values[pIdx[0]];
	float b = coefficients->values[pIdx[1]];
	float c = coefficients->values[pIdx[2]];
	float d = coefficients->values[3];

	//Find boundary of planar
	cv::Rect bo = cv::boundingRect( cv::Mat(polygon) );

	//fit planar co-ordinates to GRIDs
	float grid_step = 500;
	int center_step_x = int((bo.x + bo.width/2)/grid_step);
	int center_step_y = int((bo.y + bo.height/2)/grid_step);
	int w_step_2 = int(bo.width/(2*grid_step))+1;
	int h_step_2 = int(bo.height/(2*grid_step))+1;
	int origin_step_x = center_step_x-w_step_2;
	int origin_step_y = center_step_y-h_step_2;

	for (int i=0; i<w_step_2*2+1; i++)
	{
		for (int j=0; j<h_step_2*2+1; j++)
		{
			float x = (origin_step_x+i)*grid_step;
			float y = (origin_step_y+j)*grid_step;
			if (i%2!=0)		y = (origin_step_y + h_step_2*2 - j)*grid_step;

			int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(x,y), false );

			//Check blocking
			bool in_Blocking = checkBlocking(cv::Point2f(x,y),blockPolygons);
			if (in_Polygon==1 && !in_Blocking)
			{
				PointT pclPoint;
				pclPoint.data[pIdx[0]] = x;
				pclPoint.data[pIdx[1]] = y;
				pclPoint.data[pIdx[2]] = (- d - a*x - b*y )/c;
				result_cloud->push_back(pclPoint);
			}
		}
	}
	return result_cloud;
}

PointCloudPtrT createWP_other(const pcl::ModelCoefficients::Ptr coefficients, const pcl::ModelCoefficients::Ptr prjCoff, 
							  const PointCloudPtrT boundary,  std::vector<PointCloudT> & blockBoundary,
							  TYPE_PROJECTED_PLANE type_plane, float capture_width, float capture_height, float attitude)
{
	using namespace Eigen;
	PointCloudPtrT result_cloud(new PointCloudT);
	
	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	std::vector<cv::Point2f> polygon;
	std::vector<std::vector<cv::Point2i>> blockPolygons;
	switch (type_plane)
	{
	case TYPE_PROJECTED_PLANE_001:
		{
			double k = -a/b;			// y=k*x+t
			if(abs(c)>0.999)	k=0;	//check if c==1 and a & b is too small the estimated MODEL by RANSAC may be inaccurate
			double r_angle = - atan(k);
			double c_width = capture_width*fabs(c);
			double c_height = capture_height*fabs(c);
			Eigen::Affine3d rm = Eigen::Affine3d::Identity();
			rm.rotate(AngleAxisd(r_angle, Vector3d::UnitZ()));
			Eigen::Affine3d rm_inverted = Eigen::Affine3d::Identity();
			rm_inverted.rotate(AngleAxisd(-r_angle, Vector3d::UnitZ()));

			//Rotate Boundary + Generate Polygon
			PointCloudPtrT boundary_rotated(new PointCloudT);
			pcl::transformPointCloud(*boundary,*boundary_rotated,rm);
			std::vector<cv::Point2f> polygon;
			for (int i=0; i<boundary_rotated->size(); i++)
			{
				PointT p = boundary_rotated->points[i];
				polygon.push_back(cv::Point2f(p.x,p.y));
			}

			//Create block polygons
			PointCloudPtrT block_boundary_rotated(new PointCloudT);
			for (int i=0; i<blockBoundary.size(); i++)
			{
				pcl::transformPointCloud(blockBoundary[i],*block_boundary_rotated,rm);
				std::vector<cv::Point2i> plg = convertPolygon3Dto2D(block_boundary_rotated,prjCoff);
				blockPolygons.push_back(plg);
				block_boundary_rotated->clear();
			}

			//Generate Cloud x-y-0
			cv::Rect box = cv::boundingRect(cv::Mat(polygon));
			double max_width_step = box.width/c_width+1;
			double max_height_step = box.height/c_height+1;

			for (int i=0; i<max_height_step; i++)
			{
				double y = box.y+i*c_height;
				for (int j=0; j<max_width_step; j++)
				{
					double x;
					(i%2==0) ? x=box.x+j*c_width : x=box.x+(max_width_step-j)*c_width;
					int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(x,y), false );
					bool in_Blocking = checkBlocking(cv::Point2f(x,y),blockPolygons);
					if (in_Polygon==1 && !in_Blocking)	result_cloud->push_back(PointT(x,y,0));
				}
			}

			//Rotate back to real coordinates
			pcl::transformPointCloud(*result_cloud,*result_cloud,rm_inverted);
			for (int i=0; i<result_cloud->size(); i++){
				PointT p = result_cloud->points[i];
				result_cloud->points[i].z = ( - d - a* p.x - b* p.y )/c + c*attitude;
				//Calculate Z before add offset = (a,b,c)*attitude
				result_cloud->points[i].x += a*attitude;
				result_cloud->points[i].y += b*attitude;
			}
		}
		break;

	case TYPE_PROJECTED_PLANE_010:
		{
			for (int i=0; i<boundary->size(); i++)	polygon.push_back(cv::Point2f(boundary->points[i].x,boundary->points[i].z));
			
			for (int i=0; i<blockBoundary.size(); i++)	blockPolygons.push_back(convertPolygon3Dto2D(blockBoundary[i].makeShared(),prjCoff));

			//Generate Cloud x-0-z
			double c_width = capture_width*fabs(b);
			double c_height = capture_height*fabs(b);
			cv::Rect box = cv::boundingRect(cv::Mat(polygon));
			double max_width_step = box.width/c_width+1;
			double max_height_step = box.height/c_height+1;

			for (int i=0; i<max_height_step; i++)
			{
				double z = box.y+i*c_height;
				for (int j=0; j<max_width_step; j++)
				{
					double x;
					(i%2==0) ? x=box.x+j*c_width : x=box.x+(max_width_step-j)*c_width;
					int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(x,z), false );
					if (in_Polygon==1)
					{
						//xyz plane + offset (a,b,c)*attitude
						result_cloud->push_back(PointT(x+a*attitude,	(- d - a*x - c*z)/b +b*attitude,	z+c*attitude)); 
					}
				}
			}
		}
		break;

	case TYPE_PROJECTED_PLANE_100:
		{
			for (int i=0; i<boundary->size(); i++)	polygon.push_back(cv::Point2f(boundary->points[i].y,boundary->points[i].z));
			for (int i=0; i<blockBoundary.size(); i++)	blockPolygons.push_back(convertPolygon3Dto2D(blockBoundary[i].makeShared(),prjCoff));
			//Generate Cloud 0-y-z
			cv::Rect box = cv::boundingRect(cv::Mat(polygon));
			double c_width = capture_width*fabs(a);
			double c_height = capture_height*fabs(a);
			double max_width_step = box.width/c_width+1;
			double max_height_step = box.height/c_height+1;

			for (int i=0; i<max_height_step; i++)
			{
				double z = box.y+i*c_height;
				for (int j=0; j<max_width_step; j++)
				{
					double y;
					(i%2==0) ? y=box.x+j*c_width : y=box.x+(max_width_step-j)*c_width;
					//double y = box.x+j*c_width;
					int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(y,z), false );
					if (in_Polygon==1)
						//xyz plane + offset (a,b,c)*attitude
						result_cloud->push_back(PointT( (- d - b*y - c*z)/a	+a*attitude,	y+b*attitude,	z+c*attitude) );  
				}
			}
		}
		break;

	default:
		break;
	}	

	return result_cloud;
}

void getRotationMatrix(Eigen::Vector3d N1, Eigen::Vector3d N2, Eigen::Affine3d & rot_N1_N2, Eigen::Affine3d & rot_N2_N1)
{
	N1.normalize();
	N2.normalize();

	Eigen::Vector3d rot_axis(N1.cross(N2));
	rot_axis.normalize();

	double cos_angle = N1.dot(N2);
	double angle = -acos(cos_angle);
	rot_N2_N1 = Eigen::AngleAxisd(angle, rot_axis);
	rot_N1_N2 = Eigen::AngleAxisd(-angle, rot_axis);
}

//PointCloudPtrT createCapturePoints(const pcl::ModelCoefficients::Ptr coefficients, 
//								   const PointCloudPtrT boundary, 
//								   float capture_width, float capture_height, float attitude)
//{
//	PointCloudPtrT result_cloud(new PointCloudT);
//	float a = coefficients->values[0];
//	float b = coefficients->values[1];
//	float c = coefficients->values[2];
//	float d = coefficients->values[3];
//	Eigen::Vector3d planeNorm(a, b, c);
//	Eigen::Vector3d standardNorm(0,0,1);
//	Eigen::Affine3d toStandard;
//	Eigen::Affine3d toPlane;
//
//	getRotationMatrix(planeNorm,standardNorm,toStandard,toPlane);
//
//	//Rotate Boundary + Generate Polygon
//	PointCloudPtrT boundary_rotated(new PointCloudT);
//	pcl::transformPointCloud(*boundary,*boundary_rotated,toStandard);
//	std::vector<cv::Point2f> polygon;
//	float sigma_z=0;
//	for (int i=0; i<boundary_rotated->size(); i++)
//	{
//		PointT p = boundary_rotated->points[i];
//		polygon.push_back(cv::Point2f(p.x,p.y));
//		sigma_z+=p.z;
//	}
//	float costants_z = sigma_z/boundary_rotated->size() + attitude;
//
//	//Generate Cloud x-y-0
//	cv::Rect box = cv::boundingRect(cv::Mat(polygon));
//	double max_width_step = box.width/capture_width+1;
//	double max_height_step = box.height/capture_height+1;
//
//	for (int i=0; i<max_height_step; i++)
//	{
//		double y = box.y+i*capture_height;
//		for (int j=0; j<max_width_step; j++)
//		{
//			double x;
//			(i%2==0) ? x=box.x+j*capture_width : x=box.x+(max_width_step-j)*capture_width;
//			int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(x,y), false );
//			if (in_Polygon==1)	result_cloud->push_back(PointT(x,y,costants_z));
//		}
//	}
//
//	//Rotate back to real coordinates
//	pcl::transformPointCloud(*result_cloud,*result_cloud,toPlane);
//	return result_cloud;
//}

PointCloudPtrT helper::generate_CapturePoint(const pcl::ModelCoefficients::Ptr coefficients, const PointCloudPtrT boundary, std::vector<PointCloudT> & blockBoundary,
											 float capture_width, float capture_height, float attitude)
{
	PointCloudPtrT result_cloud(new PointCloudT);

	Eigen::Vector3i pIdx(0,1,2);//Index of X-Y-Z

	Eigen::Vector3f norm_model(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
	pcl::ModelCoefficients::Ptr prjCoff = estimateBestNorm(norm_model,pIdx);

	TYPE_PROJECTED_PLANE type_plane = TYPE_PROJECTED_PLANE_001;
	if (prjCoff->values[0]>0) type_plane = TYPE_PROJECTED_PLANE_100;
	if (prjCoff->values[1]>0) type_plane = TYPE_PROJECTED_PLANE_010;

	return createWP_other(coefficients,prjCoff,
		boundary,blockBoundary,
		type_plane,capture_width,capture_height,attitude);
}



PointCloudPtrT generateNegativeMap(PointCloudPtrT planeCloud, pcl::ModelCoefficients planeModel, PointCloudPtrT boundary)
{
	float a = planeModel.values[0];
	float b = planeModel.values[1];
	float c = planeModel.values[2];
	float d = planeModel.values[3];

	Eigen::Vector3d planeNorm(a, b, c);
	Eigen::Vector3d standardNorm(0,0,1);
	Eigen::Affine3d toStandard;
	Eigen::Affine3d toPlane;
	getRotationMatrix(planeNorm,standardNorm,toStandard,toPlane);

	PointCloudPtrT planeCloud_Rotated(new PointCloudT);
	PointCloudPtrT boundary_Rotated(new PointCloudT);
	pcl::transformPointCloud(*planeCloud,*planeCloud_Rotated,toStandard);
	pcl::transformPointCloud(*boundary,*boundary_Rotated,toStandard);

	PointCloudPtrT blockingCloud_Rotated(new PointCloudT);
	PointCloudPtrT blockingCloud(new PointCloudT);

	//Calculate Polygon
	std::vector<cv::Point2f> polygon;
	float sigma_z=0;
	for (int i=0; i<boundary_Rotated->size(); i++)
	{
		PointT p = boundary_Rotated->points[i];
		polygon.push_back(cv::Point2f(p.x,p.y));
		sigma_z+=p.z;
	}
	float costants_z = sigma_z/boundary_Rotated->size();

	//Generate Cloud x-y-0
	double step_size = 100; //100mm
	double min_square_Distance = step_size*step_size;

	cv::Rect box = cv::boundingRect(cv::Mat(polygon));
	double max_width_step = box.width/step_size + 1;
	double max_height_step = box.height/step_size + 1;

	// K nearest neighbor search
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (planeCloud_Rotated);

	for (int i=0; i<max_height_step; i++)
	{
		double y = box.y+i*step_size;
		for (int j=0; j<max_width_step; j++)
		{
			double x = box.x+j*step_size;
			int in_Polygon = cv::pointPolygonTest( polygon, cv::Point2f(x,y), false );

			if (in_Polygon==1)
			{
				//Find Negative Points
				PointT searchPoint = PointT(x,y,costants_z);
				pointIdxNKNSearch.clear();
				pointNKNSquaredDistance.clear();
				if ( kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance) > 0 )
				{
					if (pointNKNSquaredDistance[0] > min_square_Distance)	
						blockingCloud_Rotated->push_back(PointT(x,y,costants_z));
				}
			}
		}
	}

	pcl::transformPointCloud(*blockingCloud_Rotated,*blockingCloud,toPlane);
	return blockingCloud;
}

PointCloudPtrT concaveHullEsitmate(PointCloudPtrT cloud_input)
{
	PointCloudPtrT cloud_hull (new PointCloudT);
	pcl::ConcaveHull<PointT> chull;
	chull.setInputCloud (cloud_input);
	chull.setAlpha(150);
	chull.reconstruct (*cloud_hull);

	return cloud_hull;
}

void clusteringPlane(PointCloudPtrT inputCloud, float clusterThreshold, float min_size, float cell_size, std::vector<PointCloudT> & resultCloud)
{
	float min_size_points = min_size*1000000/(cell_size*cell_size);
	/// - Preparing for Euclidean segmentation
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (inputCloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::IndicesPtr ci;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (clusterThreshold); // 1 for 1mm // 200 mm by default
	ec.setSearchMethod (tree);
	ec.setInputCloud (inputCloud);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
	{
		if (it->indices.size()>min_size_points)
		{
			PointCloudPtrT cloud_cluster (new PointCloudT);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back (inputCloud->points[*pit]);

			resultCloud.push_back(*concaveHullEsitmate(helper::statisticalOutlierRemoval(cloud_cluster,cell_size)));
		}

	}
}

void helper::generateBlockingBoundary(PointCloudPtrT planeCloud, PointCloudPtrT hullCloud, pcl::ModelCoefficients::Ptr planeModel, std::vector<PointCloudT> & blockBoundary)
{
	PointCloudPtrT projectedCloud = helper::projectCloud(planeCloud,planeModel);
	PointCloudPtrT blockingCloud = generateNegativeMap(projectedCloud,*planeModel,hullCloud);
	clusteringPlane(blockingCloud, 200, 1, 100, blockBoundary);
}