#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/point_cloud.h>

void fromPCL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input, pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &output);