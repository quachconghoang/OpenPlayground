#ifndef WORLD_SEGMENTATION
#define WORLD_SEGMENTATION

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/gp3.h>

#include <pcl/segmentation/sac_segmentation.h>

#ifndef HOANGQC_POINTXYZ
#define HOANGQC_POINTXYZ
typedef pcl::PointXYZ							PointT;
typedef pcl::PointCloud<pcl::PointXYZ>			PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr		PointCloudPtrT;

typedef pcl::Normal								NormalT;
typedef pcl::PointCloud<pcl::Normal>			NormalCloudT;
typedef pcl::PointCloud<pcl::Normal>::Ptr		NormalCloudPtrT;
#endif

enum TYPE_PROJECTED_PLANE
{
	/** \brief Projected to OXY plane */
	TYPE_PROJECTED_PLANE_001,
	/** \brief Projected to OXZ plane */
	TYPE_PROJECTED_PLANE_010,
	/** \brief Projected to OYZ plane */
	TYPE_PROJECTED_PLANE_100
};

namespace helper
{
	using namespace pcl;
	using namespace pcl::visualization;

	PointCloudPtrT downSampling(PointCloudPtrT & targetCloud, float leafSize);

	PointCloudPtrT statisticalOutlierRemoval(PointCloudPtrT cloud_input, int neighbors);

	PointCloudPtrT projectCloud(PointCloudPtrT cloud_input, pcl::ModelCoefficients::Ptr coefficients);

	PointCloudPtrT hullEsitmate(PointCloudPtrT cloud_input);

	pcl::PolygonMesh::Ptr triangulation(PointCloudPtrT cloud_input);

	PointCloudPtrT generateDisplayCloud(const pcl::ModelCoefficients::Ptr coefficients, 
		const PointCloudPtrT boundary, 
		const std::vector<PointCloudT> & blockBoundary);

	PointCloudPtrT generate_CapturePoint(const pcl::ModelCoefficients::Ptr coefficients, 
		const PointCloudPtrT boundary, std::vector<PointCloudT> & blockBoundary, 
		float capture_width, float capture_height, float attitude);

}


class WorldSegmentation
{
public:
	WorldSegmentation();
	~WorldSegmentation();

};
#endif


