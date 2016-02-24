#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/point_cloud.h>

void fromPCL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input, pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &output)
{
	output->points.resize(input->points.size());
	for (size_t i = 0; i < input->points.size (); ++i) {
		pcl::cuda::PointXYZRGB pt;
		pt.x = input->points[i].x;
		pt.y = input->points[i].y;
		pt.z = input->points[i].z;
		// Pack RGB into a float
		pt.rgb = *(float*)(&input->points[i].rgb);
		output->points[i] = pt;
	}
	output->width = input->width;
	output->height = input->height;
	output->is_dense = input->is_dense;
}