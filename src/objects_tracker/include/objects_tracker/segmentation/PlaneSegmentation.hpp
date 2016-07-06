#include <objects_tracker/utilities/pcl.hpp>

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
	void computePlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void updateMask(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getCoefficients(pcl::ModelCoefficients &coefficients);
	void setCoefficients(const pcl::ModelCoefficients &coefficients);
	void computeBoundary(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void getBoundary(std::vector<pcl::PointXYZRGBA> &boundary) const;
};