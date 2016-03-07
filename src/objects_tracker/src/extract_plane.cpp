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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

// Boost
#include <boost/thread/mutex.hpp>

// My libraries
#include <objects_tracker/utilities.hpp>

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
const int NUM_IT = 1;

// Time control variables.
const int MEAN_ELEM = 25;
unsigned long total_time = 0;
unsigned long times = 0;

// Cluster variables.
const int MIN_CLUSTER_POINTS = 10000;
const int MAX_CLUSTER_POINTS = 500000;

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

void findLines(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::ModelCoefficients> &coef){
	// Cloud containing the points without the planes.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
	coef = std::vector<pcl::ModelCoefficients>(4);

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	// Set segmentation parameters.
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setIndices(inputIndices);
	seg.setOptimizeCoefficients(true);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(5000);
	seg.setDistanceThreshold(0.005);

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// At each step, one plane is removed from remainingCloud.
	for(int i = 0; i < 4; i++){

		pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud.
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers, coef[i]);

		if (inliers->indices.size() == 0) break;

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);
	}
}

void findConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, pcl::PointIndices &hullIndices) {
	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGBA>);
	hullIndices = pcl::PointIndices();
	pcl::ConvexHull<pcl::PointXYZRGBA> chull;
	chull.setInputCloud(cloud);
	chull.setIndices(inputIndices);
	chull.reconstruct(*cloud_hull);
	chull.getHullPointIndices(hullIndices); 
}

void clustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::IndicesPtr &inputIndices) {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusterIndices = std::vector<pcl::PointIndices>();

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.005);	// 1cm
	ec.setIndices(inputIndices);
	ec.setMinClusterSize(MIN_CLUSTER_POINTS);
	ec.setMaxClusterSize(MAX_CLUSTER_POINTS);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);

	if (clusterIndices.size() > 0) {

		// Find the biggest cluster.
		int max_size = clusterIndices[0].indices.size();
		int max_pos = 0;
		for(int i = 0; i < clusterIndices.size(); i++) {
			if (clusterIndices[i].indices.size() > max_size) {
				max_size = clusterIndices[i].indices.size();
				max_pos = i;
			}
		}

		// Compute the convex hull of the cluster.
		pcl::PointIndices hullIndices = pcl::PointIndices();
		pcl::PointIndices::ConstPtr clusterIndicesPtr = boost::make_shared<pcl::PointIndices>(clusterIndices[max_pos]);
		findConvexHull(cloud, clusterIndicesPtr, hullIndices);
		pcl::PointIndices::ConstPtr hullIndicesPtr = boost::make_shared<pcl::PointIndices>(hullIndices);
		std::vector<pcl::ModelCoefficients> coefficients;
		findLines(cloud, hullIndicesPtr, coefficients);
		for(int i = 0; i < coefficients.size(); i++) {
			colorLine(cloud, clusterIndicesPtr, coefficients[i], 0.005, 255, 0, 0);
		}
		//colorPointCloud(cloud, hullIndices, 0, 255, 0);
	} else {
		ROS_WARN("No plane was found during clustering, the used parameters are %i and %i (minimum and maximum size)", MIN_CLUSTER_POINTS, MAX_CLUSTER_POINTS);
	}
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
	seg.setMaxIterations(5000);
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

void removePlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,const std::vector<pcl::ModelCoefficients> &coefficients, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &remainingCloud, boost::mutex &m) {

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
		/*extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);*/
		clustering(remainingCloud, inliers);
		//colorPointCloud(remainingCloud, inliers, 255, 0, 0);
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