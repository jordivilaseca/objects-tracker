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

using namespace std;
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

int NUM_IT = 8;

pcl::PointCloud<pcl::PointXYZRGBA> cloud_f;

ros::Publisher cam1_pub;
ros::Publisher cam2_pub;

long long getTime(){
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000; //get current timestamp in milliseconds
    return mslong;
}

void getPlanesCoeffcients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<pcl::ModelCoefficients::Ptr> &coefficients) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	long long init = getTime();

	coefficients = std::vector<pcl::ModelCoefficients::Ptr>();
  	std::vector<pcl::PointIndices::Ptr> inliers = std::vector<pcl::PointIndices::Ptr>();

  	for(int i = 0; i < NUM_IT; i++) {
  		coefficients.push_back(boost::make_shared<pcl::ModelCoefficients>());
  		inliers.push_back(boost::make_shared<pcl::PointIndices>());
  	}

	//std::vector<pcl::ModelCoefficients::Ptr> coefficients = std::vector<pcl::ModelCoefficients::Ptr>();
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	//seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// False => extreu pla
	int i = 0;
	while(i < NUM_IT){
		cout << "Iteration " << i << endl;
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers[i], *coefficients[i]);
		if (inliers[i]->indices.size() == 0) break;

		// Extract the inliers
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers[i]);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
		i++;
	}
	cout << "Calcular coeficients dels plans, time " << (getTime() - init) << endl;
}

void removePlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,const  std::vector<pcl::ModelCoefficients::Ptr> &coefficients, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &remainingCloud) {

	long long init = getTime();
	remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr dit(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud));

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	for(int i = 0; i < coefficients.size(); i++) {
		cout << "Iteration " << i << endl;

		Eigen::Vector4f coef = Eigen::Vector4f(coefficients[i]->values.data());
		pcl::IndicesPtr inliers = pcl::IndicesPtr(new vector<int>());
		dit->selectWithinDistance(coef, 0.01, *inliers);

		// Extract the inliers
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
	cout << "Extreure punts dels plans, time " << (getTime() - init) << endl;
}


void callback_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	std::vector<pcl::ModelCoefficients::Ptr> coefficients;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
	getPlanesCoeffcients(cloud, coefficients);
	removePlanes(cloud, coefficients, planeCloud);
	planeCloud->header.frame_id = "cam1_link";
	cam1_pub.publish(planeCloud);
}
void callback_cam2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	std::vector<pcl::ModelCoefficients::Ptr> coefficients;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
	getPlanesCoeffcients(cloud, coefficients);
	removePlanes(cloud, coefficients, planeCloud);
	planeCloud->header.frame_id = "cam2_link";
	cam2_pub.publish(planeCloud);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "extract_plane");
  	ros::NodeHandle nh;

  	// Initialize camera 1 subscribers.
  	ROS_INFO("Camera 1 subscribers: %s\n", CAM1_POINTCLOUD);
 	ros::Subscriber cam1_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &callback_cam1);

	// Initialize camera 2 subscribers.
	ROS_INFO("Camera 2 subscribers: %s\n", CAM2_POINTCLOUD);
 	ros::Subscriber cam2_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &callback_cam2);

 	// Initialize camera 1 publishers.
	ROS_INFO("Camera 1 planes PointCloud publisher: %s\n", CAM1_POINTCLOUD_PLANE);
	cam1_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD_PLANE, 1);

	// Initialize camera 2 publishers.
	ROS_INFO("Camera 2 planes PointCloud publisher: %s\n", CAM2_POINTCLOUD_PLANE);
	cam2_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD_PLANE, 1);

	ros::spin();
}