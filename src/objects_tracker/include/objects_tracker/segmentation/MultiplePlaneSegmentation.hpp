// pcl libraries.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/segmentation/PlaneSegmentation.hpp>

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
	void computePlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void updateMasks(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getCoefficients(std::vector<pcl::ModelCoefficients> &coefficients);
	void computeBoundaries(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getBoundaries(std::vector<std::vector<pcl::PointXYZRGBA>> &boundaries);
	void segment(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<pcl::PointIndices> &clusterIndices);
};