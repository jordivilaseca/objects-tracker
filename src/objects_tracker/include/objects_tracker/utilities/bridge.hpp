// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Opencv includes
#include "opencv2/core/core.hpp"

void pointcloud2mat(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, cv::Mat &image, cv::Mat &mask);