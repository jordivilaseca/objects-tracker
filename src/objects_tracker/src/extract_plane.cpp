// Ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Point cloud
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/sample_consensus/sac_model.h>
#include <pcl/cuda/sample_consensus/ransac.h>
#include <pcl/cuda/sample_consensus/sac_model_plane.h>
#include "utilities.h"

using namespace std;
using namespace pcl::cuda;
using namespace sensor_msgs;
using namespace message_filters;

#define COLOR_IMAGE "image_color_rect"
#define DEPTH_IMAGE "image_depth_rect"
#define CAMERA_INFO "camera_info"

#define CAM1 "cam1"
#define CAM2 "cam2"
#define QUALITY_CAM "qhd"

#define CAM1_QUALITY "/" CAM1 "/" QUALITY_CAM "/"
#define CAM2_QUALITY "/" CAM2 "/" QUALITY_CAM "/"

#define CAM1_COLOR CAM1_QUALITY COLOR_IMAGE
#define CAM1_DEPTH CAM1_QUALITY DEPTH_IMAGE
#define CAM1_CAMERA CAM1_QUALITY CAMERA_INFO

#define CAM2_COLOR CAM2_QUALITY COLOR_IMAGE
#define CAM2_DEPTH CAM2_QUALITY DEPTH_IMAGE
#define CAM2_CAMERA CAM2_QUALITY CAMERA_INFO

#define CAM1_POINTCLOUD CAM1_QUALITY "PointCloud"
#define CAM1_POINTCLOUD_FILTER CAM1_POINTCLOUD "/filtered"
#define CAM1_POINTCLOUD_PLANE CAM1_POINTCLOUD "/plane"

#define CAM2_POINTCLOUD CAM2_QUALITY "PointCloud"
#define CAM2_POINTCLOUD_FILTER CAM2_POINTCLOUD "/filtered"
#define CAM2_POINTCLOUD_PLANE CAM2_POINTCLOUD "/plane"

pcl::PointCloud<pcl::PointXYZRGBA> cloud_f;

ros::Publisher cam1_pub;
ros::Publisher cam2_pub;

/*// Not implemented in pcl yet. :(
void fromPCL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input, pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &output)
{
	//output->points.resize(input->points.size());
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
}*/

void getPlanes(const pcl::cuda::PointCloudAOS<pcl::cuda::Device>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planeCloud) {

	cout << "1" << endl;
	SampleConsensusModelPlane<Device>::Ptr model = SampleConsensusModelPlane<Device>::Ptr(new SampleConsensusModelPlane<Device>(cloud)); 	
	RandomSampleConsensus<Device> sac(model);

	//planeCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

	SampleConsensusModelPlane<Device>::IndicesPtr inliers = sac.getInliers();
	cout << "2" << endl;
	/*// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.02);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	int i = 0;
	int nr_points =(int) cloud->points.size();
	// While 30% of the original cloud is still there
	//while(cloud->points.size() > 0.3 * nr_points){
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		//if (inliers->indices.size() == 0) break;

		// Extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*planeCloud);
		i++;
	//}*/
}

void callback_cam1(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
	cout << "callback_cam1" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud;
	PointCloudAOS<Device>::Ptr cloudCuda;// = PointCloudAOS<Device>::Ptr(new PointCloudAOS<Device>());
	cout << "pre fromPCL" << endl;
	fromPCL(cloud, cloudCuda);
	cout << "post fromPCL" << endl;
	getPlanes(cloudCuda, planeCloud);
	cam1_pub.publish(planeCloud);
}
void callback_cam2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
	/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
	getPlanes(cloud, planeCloud);
	cam2_pub.publish(planeCloud);*/
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "extract_plane");
  	ros::NodeHandle nh;

  	// Initialize camera 1 subscribers.
  	ROS_INFO("Camera 1 subscribers: %s\n", CAM1_POINTCLOUD_FILTER);
 	ros::Subscriber cam1_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGB> >(CAM1_POINTCLOUD_FILTER, 1, &callback_cam1);

	// Initialize camera 2 subscribers.
	ROS_INFO("Camera 2 subscribers: %s\n", CAM2_POINTCLOUD_FILTER);
 	ros::Subscriber cam2_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGB> >(CAM2_POINTCLOUD_FILTER, 1, &callback_cam2);

 	// Initialize camera 1 publishers.
	ROS_INFO("Camera 1 planes PointCloud publisher: %s\n", CAM1_POINTCLOUD_PLANE);
	cam1_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD_PLANE, 1);

	// Initialize camera 2 publishers.
	ROS_INFO("Camera 2 planes PointCloud publisher: %s\n", CAM2_POINTCLOUD_PLANE);
	cam2_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD_PLANE, 1);

	ros::spin();
}