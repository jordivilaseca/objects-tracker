#include <objects_tracker/utilities/bridge.hpp>
#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/utilities/opencv.hpp>

// pcl includes.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void computeDescriptors(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices &indices, std::vector<std::vector<float>> &descriptor);