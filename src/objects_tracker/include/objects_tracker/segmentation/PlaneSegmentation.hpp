#include <objects_tracker/utilities/pcl.hpp>

/**
 * @brief Class in charge of segmenting point clouds using one plane.
 * @details The class is capable of finding a plane given an orientation of them and a point cloud. To compute the coefficients of a plane it is only needed
 * one point cloud. In order to obtain good results for the computation of the boundaries it is important to introduce different point clouds into the system. A point
 * is considered to be part of a valid object if it is above the plane, not part of it and inside its limits.
 */
class PlaneSegmentation
{
	
private:
	int size;
	float planeDistance;
	Eigen::Vector3f orientation;
	float angleThreshold;

	pcl::ModelCoefficients coefficients;
	std::vector<pcl::PointXYZRGBA> boundary;
	pcl::PointIndices::Ptr maskIndices;
	std::vector<int> maskCumulative;

public:
	PlaneSegmentation(Eigen::Vector3f orientation, float angleThreshold = 10.0, float planeDistance = 0.015);
	~PlaneSegmentation();
	void computeCoefficients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void updateMask(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getCoefficients(pcl::ModelCoefficients &coefficients);
	void setCoefficients(const pcl::ModelCoefficients &coefficients);
	void computeBoundary(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getBoundary(std::vector<pcl::PointXYZRGBA> &boundary) const;
};