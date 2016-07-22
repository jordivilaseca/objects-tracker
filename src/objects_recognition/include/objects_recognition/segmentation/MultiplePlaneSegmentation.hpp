// pcl libraries.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/segmentation/PlaneSegmentation.hpp>

/**
 * @brief Class in charge of segmenting point clouds using multiple planes.
 * @details The class is capable of finding different planes given an orientation of them and a point cloud. To compute the coefficients of a plane it is only needed
 * one point cloud. In order to obtain good results for the computation of the boundaries it is important to introduce different point clouds into the system. A point
 * is considered to be part of a valid object if it is above the plane, inside the limits of the planes and it is not part of any of the planes.
 */
class MultiplePlaneSegmentation
{

private:
	uint nPlanes;
	float planeDistance;
	std::vector<PlaneSegmentation> planes;
	Eigen::Vector3f orientation;
	float angleThreshold;

public:
	MultiplePlaneSegmentation(int nPlanes, Eigen::Vector3f orientation, float angleThreshold = 10.0, float planeDistance = 0.015);
	~MultiplePlaneSegmentation();
	void computeCoefficients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void updateMasks(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getCoefficients(std::vector<pcl::ModelCoefficients> &coefficients);
	void computeBoundaries(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getBoundaries(std::vector<std::vector<pcl::PointXYZRGBA>> &boundaries) const;
	void segment(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<pcl::PointIndices> &clusterIndices);
};