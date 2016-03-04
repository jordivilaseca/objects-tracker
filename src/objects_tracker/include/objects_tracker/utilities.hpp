#include <sys/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <ros/ros.h>

long long getTime();
void colorPoint(pcl::PointXYZRGBA &point, int r, int g, int b);
void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const boost::shared_ptr<std::vector<int> > &inliers, uint8_t r, uint8_t g, uint8_t b);
void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<pcl::PointIndices> &inliers);