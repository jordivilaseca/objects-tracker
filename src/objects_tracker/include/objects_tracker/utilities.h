#include <pcl/point_cloud.h>
#include <pcl/cuda/point_types.h>
#include <pcl/cuda/point_cloud.h>

void fromPCL(const pcl::PointCloud<pcl::cuda::PointXYZRGB>::Ptr &input, pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &output);