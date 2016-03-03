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

#include <boost/thread/mutex.hpp>

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

// Total number of planes to find.
int NUM_IT = 1;

// Time control variables.
int MEAN_ELEM = 25;
unsigned long total_time = 0;
unsigned long times = 0;

// Node publishers.
ros::Publisher cam1_pub;
ros::Publisher cam2_pub;

// Vectors containing the coefficients of the planes for each camera.
std::vector<pcl::ModelCoefficients> coefficients_cam1;
std::vector<pcl::ModelCoefficients> coefficients_cam2;

// In order to control the updating of the coefficients.
boost::mutex m1;
boost::mutex m2;

// We cannot make plane extraction until at least one time the coefficients are calculated.
bool existsPlaneCam1 = false;
bool existsPlaneCam2 = false;

long long getTime(){
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return mslong;
}

void getPlanesCoeffcients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<pcl::ModelCoefficients> &coefficients, boost::mutex &m) {
	long long init = getTime();

	// Cloud containing the points without the planes.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	// Set segmentation parameters.
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(2000);
	seg.setDistanceThreshold(0.015);

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// At each step, one plane is removed from remainingCloud.
	for(int i = 0; i < NUM_IT; i++){

		pcl::ModelCoefficients coef = pcl::ModelCoefficients();
		pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud.
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers, coef);

		// Safe copy of the coefficients to the global variable.
		m.lock();
		coefficients[i] = coef;
		m.unlock();

		if (inliers->indices.size() == 0) break;

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
	ROS_INFO("Calculated plane coefficients, total time %llu", (getTime() - init));
}

void removePlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,const  std::vector<pcl::ModelCoefficients> &coefficients, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &remainingCloud, boost::mutex &m) {

	long long init = getTime();

	// Cloud containing the points without the planes.
	remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	// Initialize plane segmentator.
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr dit(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (remainingCloud));

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	for(int i = 0; i < coefficients.size(); i++) {

		pcl::IndicesPtr inliers = pcl::IndicesPtr(new vector<int>());

		// Safe load of the coefficients of the global variable.
		m.lock();
		Eigen::Vector4f coef = Eigen::Vector4f(coefficients[i].values.data());
		m.unlock();

		// Get plane inliers using 'coef' as plane coefficients.
		dit->selectWithinDistance(coef, 0.015, *inliers);

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
	total_time += getTime()-init;
	times++;
	if (times == MEAN_ELEM) {
		ROS_INFO("Extracted plane points from point cloud %i times, mean time %lu", MEAN_ELEM, total_time/MEAN_ELEM);
		times = 0;
		total_time = 0;
	}
}


void planes_coefficients_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (cam1_pub.getNumSubscribers() > 0 or !existsPlaneCam1) {
		getPlanesCoeffcients(cloud, coefficients_cam1, m1);
		existsPlaneCam1 = true;
	}
}

void planes_coefficients_cam2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (cam2_pub.getNumSubscribers() > 0 or !existsPlaneCam2) {
		getPlanesCoeffcients(cloud, coefficients_cam2, m2);
		existsPlaneCam2 = true;
	}
}

void remove_planes_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (existsPlaneCam1 and cam1_pub.getNumSubscribers() > 0) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
		removePlanes(cloud, coefficients_cam1, planeCloud, m1);
		planeCloud->header.frame_id = "cam1_link";
		cam1_pub.publish(planeCloud);
	}
}

void remove_planes_cam2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (existsPlaneCam2 and cam2_pub.getNumSubscribers() > 0) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
		removePlanes(cloud, coefficients_cam2, planeCloud, m2);
		planeCloud->header.frame_id = "cam2_link";
		cam2_pub.publish(planeCloud);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "extract_plane");
  	ros::NodeHandle nh;

  	coefficients_cam1 = std::vector<pcl::ModelCoefficients>(NUM_IT, pcl::ModelCoefficients());
  	coefficients_cam2 = std::vector<pcl::ModelCoefficients>(NUM_IT, pcl::ModelCoefficients());

  	// Initialize camera 1 subscribers.
  	ROS_INFO("Camera 1 subscribers: %s\n", CAM1_POINTCLOUD);
 	ros::Subscriber cam1_plane_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &remove_planes_cam1);
 	ros::Subscriber cam1_remove_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &planes_coefficients_cam1);

	// Initialize camera 2 subscribers.
	ROS_INFO("Camera 2 subscribers: %s\n", CAM2_POINTCLOUD);
 	ros::Subscriber cam2_plane_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &remove_planes_cam2);
 	ros::Subscriber cam2_remove_sub = nh.subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &planes_coefficients_cam2);

 	// Initialize camera 1 publishers.
	ROS_INFO("Camera 1 planes PointCloud publisher: %s\n", CAM1_POINTCLOUD_PLANE);
	cam1_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD_PLANE, 1);

	// Initialize camera 2 publishers.
	ROS_INFO("Camera 2 planes PointCloud publisher: %s\n", CAM2_POINTCLOUD_PLANE);
	cam2_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD_PLANE, 1);

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
   	spinner.spin();
}