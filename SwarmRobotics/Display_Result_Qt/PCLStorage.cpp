#include "stdafx.h"
#include "PCLStorage.h"
#include "pcl/common/common_headers.h"
#include "PlanarCreator.h"

#define kDOWN_SIZE 50

PlaneStorage::PlaneStorage(std::string iD /* = "default.0" */)
{
	tagID = iD;
	modelCoefficients.reset(new pcl::ModelCoefficients);
	pointCloud.reset(new PointCloudT);
	hullCloud.reset(new PointCloudT);
	gridCloud.reset(new PointCloudT);
	mesh.reset(new pcl::PolygonMesh);
	capturePoints.reset(new PointCloudT);
	displayingMode = PLANE_NONE;
}

PlaneStorage::~PlaneStorage()
{
	//clear();
}

void PlaneStorage::clear()
{
	modelCoefficients.reset(new pcl::ModelCoefficients);
	pointCloud.reset(new PointCloudT);
	hullCloud.reset(new PointCloudT);
	gridCloud.reset(new PointCloudT);
	mesh.reset(new pcl::PolygonMesh);
	capturePoints.reset(new PointCloudT);
	for (int i = 0; i < blockCloud.size(); i++)
	{
		blockCloud[i].clear();
	}
	blockCloud.clear();
}

PCLStorage::PCLStorage()
{
	isSegmented = false;
	genRandom = cv::RNG(0xFFFFFFFF);
}

PCLStorage::~PCLStorage()
{
}

void PCLStorage::setInputCloud(std::string fileName)
{
	cloud_input.reset(new PointCloudT);
	pcl::io::loadPCDFile(fileName, *cloud_input);
}

void PCLStorage::clearData()
{

}

void PCLStorage::segmentParams(std::vector<double> params)
{
	segmentPointcloud(params[0], params[1], params[2], 20, 20, params[3]);
	emit visualConnector->signal_processFinish(PSWM_SEGMENT_DONE);
}

void PCLStorage::segmentPointcloud(double minPlaneArea, 
	double disThreshold, 
	double maxIter, 
	int maxRetry, 
	int minClusterSize, 
	double clusterThreshold)
{

	PointCloudPtrT cloud_blob(new PointCloudT);
	PointCloudPtrT cloud_remain_temp(new PointCloudT);
	PointCloudPtrT cloud_projected(new PointCloudT);
	pcl::copyPointCloud(*cloud_input, *cloud_blob);
	PointCloudPtrT cloud_f(new PointCloudT);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	/// - Prepare for RANSAC segmentation
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIter);
	seg.setDistanceThreshold(disThreshold);

	pcl::ExtractIndices<PointT> extract;
	int i = 0, nr_points = (int)cloud_blob->points.size();
	int fail_count = 0;
	while (cloud_blob->points.size() > minClusterSize && fail_count < maxRetry)
	{
		/// 1. Update progress bar animations
		double valPercent = (nr_points - cloud_blob->points.size()) * 100 / nr_points;
		emit visualConnector->signal_processBarUpdating(int(valPercent));

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		seg.setInputCloud(cloud_blob);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			//std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		/// 2. Extract the inliers
		extract.setInputCloud(cloud_blob);
		extract.setIndices(inliers);
		extract.setNegative(false);
		PointCloudPtrT cloud_p(new PointCloudT); //Cloud Plane 
		extract.filter(*cloud_p);

		/// 3. Save rest points
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_remain_temp.swap(cloud_f);

		/// - Preparing for Euclidean segmentation
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud_p);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::IndicesPtr ci;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(clusterThreshold); // 1 for 1mm
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_p);
		ec.extract(cluster_indices);

		bool isDetected = false;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			/// 1. Extract each cluster
			pcl::ModelCoefficients::Ptr new_coefficients(new pcl::ModelCoefficients());
			PointCloudPtrT cloud_cluster(new PointCloudT);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back(cloud_p->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			if (cloud_cluster->points.size() < minClusterSize)
			{ // too small cluster
				*cloud_remain_temp += *cloud_cluster; // reject and push it back to input point cloud
			}
			else
			{
				/// 2. Plane segmentation again for each cluster
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				pcl::SACSegmentation<PointT> seg; // Create the segmentation object
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(maxIter);
				seg.setDistanceThreshold(disThreshold);
				seg.setOptimizeCoefficients(true); // Optional		
				pcl::ExtractIndices<pcl::PointXYZ> extract; // Create the filtering object			
				seg.setInputCloud(cloud_cluster);
				seg.segment(*inliers, *new_coefficients);

				if (inliers->indices.size() == 0) break;

				// The largest dense plane of each cluster
				// Extract the inliers
				extract.setInputCloud(cloud_cluster);
				extract.setIndices(inliers);
				extract.setNegative(false);
				PointCloudPtrT cloud_p_final(new PointCloudT);
				extract.filter(*cloud_p_final);

				// Project the model inliers
				pcl::ProjectInliers<pcl::PointXYZ> proj;
				proj.setModelType(pcl::SACMODEL_PLANE);
				proj.setInputCloud(cloud_p_final);
				proj.setModelCoefficients(new_coefficients);
				proj.filter(*cloud_projected);

				double planeArea = helper::areaEstimate(cloud_projected, kDOWN_SIZE*1.5);
				qDebug() << "Area of the detected plane (m2): " << planeArea;

				/// 3. Found a good plane to add to storage 
				if (planeArea > minPlaneArea)
				{
					PlaneStorage pan(tagID + "." + std::to_string(planes.size()));
					// Save point cloud representing it
					pan.pointCloud = cloud_p_final;
					pan.area = planeArea;
					// Save the convex hull representing its border
					float thrs = 1000;
					if (pan.area < 20) thrs = 500;
					if (pan.area < 10) thrs = 250;
					if (pan.area < 2) thrs = 50;
					pan.hullCloud = helper::hullEsitmate(helper::downSampling(helper::hullEsitmate(cloud_projected), thrs));

					pan.modelCoefficients = new_coefficients;
					helper::generateBlockingBoundary(pan.pointCloud, pan.hullCloud, pan.modelCoefficients, pan.blockCloud);
					pan.gridCloud = helper::generateDisplayCloud(pan.modelCoefficients, pan.hullCloud, pan.blockCloud);
					qDebug() << "block size = " << pan.blockCloud.size();
					pan.mesh = helper::triangulation(pan.gridCloud);

					pan.color.r = genRandom.uniform(0.3, 1.0); 
					pan.color.g = genRandom.uniform(0.3, 1.0); 
					pan.color.b = genRandom.uniform(0.3, 1.0);
					planes.push_back(pan);

					//pcl::io::savePCDFileBinary("cube_"+std::to_string(planes.size())+".pcd",*cloud_projected);
					//cv::FileStorage fs("cube_"+std::to_string(planes.size())+".yaml", cv::FileStorage::WRITE);
					//fs << "Model" <<"[:" 
					//<< pan.modelCoefficients->values[0] 
					//<< pan.modelCoefficients->values[1] 
					//<< pan.modelCoefficients->values[2] 
					//<< pan.modelCoefficients->values[3] <<	"]";
					//fs.release();

					// Tell RANSAC that we found a good plane
					isDetected = true;
				}

				/// 4. Push remained points of cluster back to the input point cloud
				extract.setNegative(true);
				extract.filter(*cloud_f);
				*cloud_remain_temp += *cloud_f;
			}
		}

		if (!isDetected) fail_count++;  // RANSAC not found a good plane, so retry
		cloud_blob.swap(cloud_remain_temp); // continue to detect new plane in remained points

	}

	//cloud_remain = cloud_blob;
	isSegmented = true;
}