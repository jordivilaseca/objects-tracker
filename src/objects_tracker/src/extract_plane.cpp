// Ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
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

// C++
#include <queue>
#include <stack>

// My libraries
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/utilities/ros.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

#define COLOUR_IMAGE "image_colour_rect"
#define DEPTH_IMAGE "image_depth_rect"
#define CAMERA_INFO "camera_info"

#define CAM1 "cam1"
#define CAM2 "cam2"
#define QUALITY_CAM "qhd"

#define CAM1_QUALITY "/" CAM1 "/" QUALITY_CAM "/"
#define CAM2_QUALITY "/" CAM2 "/" QUALITY_CAM "/"

#define CAM1_COLOuR CAM1_QUALITY COLOUR_IMAGE
#define CAM1_DEPTH CAM1_QUALITY DEPTH_IMAGE
#define CAM1_CAMERA CAM1_QUALITY CAMERA_INFO

#define CAM2_COLOUR CAM2_QUALITY COLOUR_IMAGE
#define CAM2_DEPTH CAM2_QUALITY DEPTH_IMAGE
#define CAM2_CAMERA CAM2_QUALITY CAMERA_INFO

#define CAM1_POINTCLOUD CAM1_QUALITY "PointCloud"
#define CAM1_POINTCLOUD_FILTER CAM1_POINTCLOUD "/filtered"
#define CAM1_POINTCLOUD_PLANE CAM1_POINTCLOUD "/plane"
#define CAM1_POINTCLOUD_PLANE_MARKERS CAM1_POINTCLOUD_PLANE "/markers"

#define CAM2_POINTCLOUD CAM2_QUALITY "PointCloud"
#define CAM2_POINTCLOUD_FILTER CAM2_POINTCLOUD "/filtered"
#define CAM2_POINTCLOUD_PLANE CAM2_POINTCLOUD "/plane"
#define CAM2_POINTCLOUD_PLANE_MARKERS CAM2_POINTCLOUD_PLANE "/markers"

boost::shared_ptr<ros::NodeHandle> nh;

// Total number of planes to find.
const int NUM_IT = 2;

// Time control variables.
const int MEAN_ELEM = 25;
unsigned long total_time = 0;
unsigned long times = 0;

// Node publishers.
ros::Publisher cam1_planes_pub;
ros::Publisher cam1_marker_pub;
ros::Publisher cam2_planes_pub;
ros::Publisher cam2_marker_pub;

// Node subscribers.
ros::Subscriber cam1_plane_sub;
ros::Subscriber cam1_limits_sub; 
ros::Subscriber cam1_coef_sub;
ros::Subscriber cam2_plane_sub;
ros::Subscriber cam2_limits_sub;
ros::Subscriber cam2_remove_sub;

// Masks (one for each plane)
std::vector<pcl::PointIndices> mask_points_cam1;
std::vector< std::vector<int> > mask_cumulative_cam1;

// Vectors containing the coefficients of the planes for each camera.
std::vector<pcl::ModelCoefficients> coefficients_cam1;
std::vector<pcl::ModelCoefficients> coefficients_cam2;

// Vector containing the points that define the boundary for each plane.
std::vector<pcl::PointIndices> limits_cam1;
std::vector<pcl::PointIndices> limits_cam2;

// In order to control the updating of the coefficients.
boost::mutex m1_coef;
boost::mutex m2_coef;

boost::mutex m1_limits;
boost::mutex m2_limits;

// We cannot make plane extraction until at least one time the coefficients are calculated.
bool existsPlaneCam1 = false;
bool existsPlaneCam2 = false;

int getPlaneLimits(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, const pcl::ModelCoefficients &planeCoefficients, pcl::PointIndices &planeLimits) {

	std::vector<pcl::PointIndices> clusterIndices;
	clustering(cloud, inputIndices, clusterIndices);

	if (clusterIndices.size() == 0) return 0;

	// Find the biggest cluster.
	int max_size = clusterIndices[0].indices.size();
	int max_pos = 0;
	for(int i = 0; i < clusterIndices.size(); i++) {
		if (clusterIndices[i].indices.size() > max_size) {
			max_size = clusterIndices[i].indices.size();
			max_pos = i;
		}
	}

	// Project pointcloud to a plane.
	pcl::PointIndices::ConstPtr clusterIndicesPtr = boost::make_shared<pcl::PointIndices>(clusterIndices[max_pos]);
	pcl::ModelCoefficients::ConstPtr coefPtr = boost::make_shared<pcl::ModelCoefficients>(planeCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projectedCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	projectToPlane(cloud, coefPtr, projectedCloud);

	// Compute the convex hull of the cluster.
	pcl::PointIndices hullIndices = pcl::PointIndices();
	findConvexHull(projectedCloud, clusterIndicesPtr, hullIndices);
	pcl::PointIndices::ConstPtr hullIndicesPtr = boost::make_shared<pcl::PointIndices>(hullIndices);

	// Simplify convex polygon.
	planeLimits = pcl::PointIndices();
	polygonSimplification(cloud, hullIndicesPtr, planeCoefficients.values, 4, planeLimits);
	return planeLimits.indices.size();
}

void getPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, int nplanes, std::vector<pcl::ModelCoefficients> &coefficients, std::vector<pcl::PointIndices::Ptr> &inliers) {
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
	seg.setDistanceThreshold(0.02);

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	coefficients = std::vector<pcl::ModelCoefficients>(nplanes);
	inliers = std::vector<pcl::PointIndices::Ptr>(nplanes, boost::make_shared<pcl::PointIndices>());
	// At each step, one plane is removed from remainingCloud.
	for(int i = 0; i < nplanes; i++){

		// Segment the largest planar component from the remaining cloud.
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers[i], coefficients[i]);

		if (inliers[i]->indices.size() == 0) break;

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers[i]);
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
	//pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr dit(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (remainingCloud));

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	for(int i = 0; i < coefficients.size(); i++) {

		pcl::IndicesPtr inliers = pcl::IndicesPtr(new vector<int>());

		// Safe load of the coefficients of the global variable.
		m.lock();
		pcl::ModelCoefficients modelCoef = coefficients[i];
		Eigen::Vector4f coef = Eigen::Vector4f(modelCoef.values.data());
		m.unlock();

		findPlaneInliers(remainingCloud, modelCoef, 0.02, inliers);
		// Get plane inliers using 'coef' as plane coefficients.
		//dit->selectWithinDistance(coef, 0.02, *inliers);

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

void calculate_limits_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
//void remove_planes_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

void planes_coefficients_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (cam1_planes_pub.getNumSubscribers() > 0 or !existsPlaneCam1) {

		// Get plane coefficients and indices.
		std::vector<pcl::ModelCoefficients> planesCoefficients;
		std::vector<pcl::PointIndices::Ptr> planesIndices;
		getPlanes(cloud, NUM_IT, planesCoefficients, planesIndices);

		// Safe copy of the coefficients to the global variable.
		for(int i = 0; i < coefficients_cam1.size(); i++) {
			m1_coef.lock();
			coefficients_cam1[i] = planesCoefficients[i];
			m1_coef.unlock();
		}
		existsPlaneCam1 = true;
		cam1_limits_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &calculate_limits_cam1);
		cam1_coef_sub.shutdown();
		ROS_INFO("Calculated coefficients camera 1");
	}
}

int cam1_iter = 0;
int max_iter = 50;
void calculate_limits_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

	// Initialization
	if(cam1_iter == 0) {
		mask_cumulative_cam1 = std::vector< std::vector<int> >(NUM_IT);
		for(int i = 0; i < NUM_IT; i++) {
			mask_cumulative_cam1[i].resize(cloud->height*cloud->width, 0);
		}
	}

	// Enough iterations, it's time to calculate the limits.
	if(cam1_iter == max_iter) {
		std::vector< std::vector< std::vector<double> > > positions = std::vector< std::vector< std::vector<double> > >(mask_points_cam1.size());

		// Find the points that define the limit of each plane.
		for(int i = 0; i < mask_points_cam1.size(); i++) {
			pcl::PointIndices planeLimits = pcl::PointIndices();
			pcl::PointIndices::ConstPtr planeIndicesConstPtr = boost::make_shared<pcl::PointIndices>(mask_points_cam1[i]);
			int nplanes = getPlaneLimits(cloud, planeIndicesConstPtr, coefficients_cam1[i], planeLimits);

			// Safe copy of the limits.
			m1_limits.lock();
			limits_cam1[i] = planeLimits;
			m1_limits.unlock();

			// Calculate point position.
			for(int j = 0; j < planeLimits.indices.size(); j++) {
				int ppos = planeLimits.indices[j];
				pcl::PointXYZRGBA p = cloud->points[ppos];

				std::vector<double> pos = std::vector<double>(3);
				pos[0] = p.x;
				pos[1] = p.y;
				pos[2] = p.z;
				positions[i].push_back(pos);
			}
		}

		// Publish markers.
		if (cam1_marker_pub.getNumSubscribers() > 0) {

			uint32_t type = visualization_msgs::Marker::SPHERE;
			double width = 0.03;
			std::vector<visualization_msgs::Marker> markers;

			// Add the line between first and last point.
			for(int i = 0; i < positions.size(); i++) {
				positions[i].push_back(positions[i][0]);
			}

			// Construct line markers.
			buildLineMarkers("cam1_link", positions, width, markers);

			// Publish markers.
			for(int i = 0; i < positions.size(); i++) {
				cam1_marker_pub.publish(markers[i]);
			}
		}

		//cam1_plane_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &remove_planes_cam1);
		//cam1_limits_sub.shutdown();
		cam1_iter = 0;
		ROS_INFO("Calculated limits camera 1");
	}


	// Update planes mask.
	for(int i = 0; i < NUM_IT; i++) {

		// Get plane inliers.
		pcl::IndicesPtr inliers;
		findPlaneInliers(cloud, coefficients_cam1[i], 0.02, inliers);

		// Add new inliers and update old ones.
		for(int j = 0; j < inliers->size(); j++) {
			int p = (*inliers)[j];
			if(mask_cumulative_cam1[i][p] == 0) {
				mask_points_cam1[i].indices.push_back(p);
			}
			mask_cumulative_cam1[i][p]++;
		}
	}
	cam1_iter++;
}

void planes_coefficients_cam2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (cam2_planes_pub.getNumSubscribers() > 0 or !existsPlaneCam2) {
		std::vector<pcl::ModelCoefficients> planesCoefficients;
		std::vector<pcl::PointIndices::Ptr> planesIndices;
		getPlanes(cloud, NUM_IT, planesCoefficients, planesIndices);

		// Safe copy of the coefficients to the global variable.
		for(int i = 0; i < coefficients_cam2.size(); i++) {
			m2_coef.lock();
			coefficients_cam2[i] = planesCoefficients[i];
			m2_coef.unlock();
		}

		// Find the points that define the limit of each plane.
		for(int i = 0; i < planesIndices.size(); i++) {
			pcl::PointIndices planeLimits;
			int nplanes = getPlaneLimits(cloud, planesIndices[i], planesCoefficients[i], planeLimits);

			// Safe copy of the limits.
			m2_limits.lock();
			limits_cam2[i] = planeLimits;
			m2_limits.unlock();
			
			// Publish markers.
			if (cam2_marker_pub.getNumSubscribers() > 0) {
				for(int j = 0; j < planeLimits.indices.size(); j++) {
					int ppos = planeLimits.indices[j];
					pcl::PointXYZRGBA p = cloud->points[ppos];

					double pos[] = {p.x, p.y, p.z};
					double scale[] = {0.02,0.02,0.02};
					double colour[] = {0, 255, 0, 1};
					visualization_msgs::Marker marker = buildMarker("cam2_link", j, visualization_msgs::Marker::CUBE, pos, scale, colour);
					cam2_marker_pub.publish(marker);
				}
			}
		}
		existsPlaneCam2 = true;
	}
}

void remove_planes_cam1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (existsPlaneCam1 and cam1_planes_pub.getNumSubscribers() > 0) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
		removePlanes(cloud, coefficients_cam1, planeCloud, m1_coef);
		planeCloud->header.frame_id = "cam1_link";
		cam1_planes_pub.publish(planeCloud);
	}
}

void remove_planes_cam2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	if (existsPlaneCam2 and cam2_planes_pub.getNumSubscribers() > 0) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud;
		removePlanes(cloud, coefficients_cam2, planeCloud, m2_coef);
		planeCloud->header.frame_id = "cam2_link";
		cam2_planes_pub.publish(planeCloud);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "extract_plane");
	nh.reset(new ros::NodeHandle());

  	coefficients_cam1 = std::vector<pcl::ModelCoefficients>(NUM_IT, pcl::ModelCoefficients());
  	coefficients_cam2 = std::vector<pcl::ModelCoefficients>(NUM_IT, pcl::ModelCoefficients());

  	limits_cam1 = std::vector<pcl::PointIndices>(NUM_IT, pcl::PointIndices());
  	limits_cam2 = std::vector<pcl::PointIndices>(NUM_IT, pcl::PointIndices());

  	mask_points_cam1 = std::vector<pcl::PointIndices>(NUM_IT, pcl::PointIndices());
	mask_cumulative_cam1 = std::vector< std::vector<int> >(NUM_IT);

  	// Initialize camera 1 subscribers.
  	ROS_INFO("Camera 1 subscribers: %s\n", CAM1_POINTCLOUD);
 	cam1_plane_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &remove_planes_cam1);
 	//cam1_limits_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &calculate_limits_cam1);
 	cam1_coef_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD, 1, &planes_coefficients_cam1);

	// Initialize camera 2 subscribers.
	ROS_INFO("Camera 2 subscribers: %s\n", CAM2_POINTCLOUD);
 	cam2_plane_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &remove_planes_cam2);
 	//cam2_limits_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &calculate_limits_cam2);
 	cam2_remove_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD, 1, &planes_coefficients_cam2);

 	// Initialize camera 1 publishers.
	ROS_INFO("Camera 1 PointCloud planes publisher: %s\n", CAM1_POINTCLOUD_PLANE);
	cam1_planes_pub = nh->advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM1_POINTCLOUD_PLANE, 1);

	ROS_INFO("Camera 1 planes markers publisher: %s\n", CAM1_POINTCLOUD_PLANE_MARKERS);
	cam1_marker_pub = nh->advertise<visualization_msgs::Marker>(CAM1_POINTCLOUD_PLANE_MARKERS, 1);

	// Initialize camera 2 publishers.
	ROS_INFO("Camera 2 PointCloud planes publisher: %s\n", CAM2_POINTCLOUD_PLANE);
	cam2_planes_pub = nh->advertise< pcl::PointCloud<pcl::PointXYZRGBA> >(CAM2_POINTCLOUD_PLANE, 1);

	ROS_INFO("Camera 2 planes markers publisher: %s\n", CAM2_POINTCLOUD_PLANE_MARKERS);
	cam2_marker_pub = nh->advertise<visualization_msgs::Marker>(CAM2_POINTCLOUD_PLANE_MARKERS, 1);

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
   	spinner.spin();
}