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
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

// Boost
#include <boost/thread/mutex.hpp>

// My libraries
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/utilities/ros.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

#define CAM1 "cam1"
#define CAM2 "cam2"
#define QUALITY_CAM "qhd"

boost::shared_ptr<ros::NodeHandle> nh;

struct kinect {

	struct plane {
		pcl::PointIndices mask_points;
		std::vector<int> mask_cumulative;
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices limits;
	};
	int nplanes;
	int iter;

	std::string id;
	std::string frame_id;
	std::string pub_marker_channel;
	std::string pub_plane_channel;
	std::string sub_channel;

	ros::Publisher plane_pub;
	ros::Publisher marker_pub;
	ros::Subscriber sub;

	std::vector<plane> planes;

	boost::mutex m_coef;
	boost::mutex m_limits;

	bool existsCoefficients;
	bool existsLimits;

	kinect() {}

	void init(std::string id, std::string fid, std::string quality, int n) {
		nplanes = n;
		this->id = id;
		frame_id = fid;
		iter = 0;
		pub_plane_channel = "/" + id + "/" + quality + "/PointCloud/plane";
		pub_marker_channel = pub_plane_channel + "/markers";
		sub_channel = "/" + id + "/" + quality + "/PointCloud";

		planes = std::vector<plane>(nplanes);
	}
};

// Total number of planes to find.
const int NUM_PLANES = 2;

const int NUM_CAMS = 2;

const int MAX_LIMIT_ITER = 50;

// Time control variables.
const int MEAN_ELEM = 25;
unsigned long total_time = 0;
unsigned long times = 0;

std::vector<kinect> ks;

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
	polygonSimplification(projectedCloud, hullIndicesPtr, planeCoefficients.values, 4, planeLimits);
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

void remove_planes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	if (k.plane_pub.getNumSubscribers() > 0) {
		// Cloud containing the points without the planes.
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
		
		long long init = getTime();

		// Create the filtering object.
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

		for(int i = 0; i < k.nplanes; i++) {

			pcl::IndicesPtr inliers = pcl::IndicesPtr(new vector<int>());

			// Safe load of the coefficients of the global variable.
			k.m_coef.lock();
			pcl::ModelCoefficients planeCoef = k.planes[i].coefficients;
			Eigen::Vector4f coef = Eigen::Vector4f(planeCoef.values.data());
			k.m_coef.unlock();

			k.m_limits.lock();
			pcl::PointIndices planeLimits = k.planes[i].limits;
			k.m_limits.unlock();

			findPlaneInliers(remainingCloud, planeCoef, planeLimits, 0.02, inliers);

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

		remainingCloud->header.frame_id = k.frame_id;
		k.plane_pub.publish(remainingCloud);
	}
}

void initialize_limits(kinect& k, int size) {
	k.iter = 0;
	for(int i = 0; i < NUM_PLANES; i++) {
		k.planes[i].mask_cumulative.resize(size, 0);
		k.planes[i].mask_points = pcl::PointIndices();
	}
}

void update_limits(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	// Update planes mask.
	for(int i = 0; i < NUM_PLANES; i++) {

		// Get plane inliers.
		pcl::IndicesPtr inliers;
		findPlaneInliers(cloud, k.planes[i].coefficients, 0.02, inliers);

		// Add new inliers and update old ones.
		for(int j = 0; j < inliers->size(); j++) {
			int p = (*inliers)[j];
			if(k.planes[i].mask_cumulative[p] == 0) {
				k.planes[i].mask_points.indices.push_back(p);
			}
			k.planes[i].mask_cumulative[p]++;
		}
	}
	k.iter++;
}

void finish_limits(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {

	// Find the points that define the limit of each plane.
	for(int i = 0; i < k.nplanes; i++) {
		pcl::PointIndices planeLimits = pcl::PointIndices();
		pcl::PointIndices::ConstPtr planeIndicesConstPtr = boost::make_shared<pcl::PointIndices>(k.planes[i].mask_points);
		int nplanes = getPlaneLimits(cloud, planeIndicesConstPtr, k.planes[i].coefficients, planeLimits);

		// Safe copy of the limits.
		k.m_limits.lock();
		k.planes[i].limits = planeLimits;
		k.m_limits.unlock();
	}
	ROS_INFO("Calculated limits %s", k.id.c_str());
}

void publish_limits(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	if (k.marker_pub.getNumSubscribers() > 0) {
		std::vector< std::vector< std::vector<double> > > positions = std::vector< std::vector< std::vector<double> > >(k.nplanes);
		for(int i = 0; i < k.nplanes; i++) {
			for(int j = 0; j < k.planes[i].limits.indices.size(); j++) {
				pcl::PointIndices planeLimits = k.planes[i].limits;
				int ppos = planeLimits.indices[j];

				pcl::PointXYZRGBA p = cloud->points[ppos];
				std::vector<double> pos = std::vector<double>(3);
				pos[0] = p.x;
				pos[1] = p.y;
				pos[2] = p.z;
				positions[i].push_back(pos);
			}
		}

		double width = 0.03;
		std::vector<visualization_msgs::Marker> markers;

		// Add the line between first and last point.
		for(int i = 0; i < positions.size(); i++) {
			positions[i].push_back(positions[i][0]);
		}

		// Construct line markers.
		buildLineMarkers(k.frame_id, positions, width, markers);

		// Publish markers.
		for(int i = 0; i < positions.size(); i++) {
			k.marker_pub.publish(markers[i]);
		}
	}

}

void calculate_limits(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {

	// Initialization
	if(k.iter == 0) {
		initialize_limits(k, cloud->height*cloud->width);
	}

	update_limits(cloud, k);

	// Enough iterations, it's time to calculate the limits.
	if(k.iter == MAX_LIMIT_ITER) {
		finish_limits(cloud, k);
		publish_limits(cloud, k);
		k.sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(k.sub_channel, 1, boost::bind(remove_planes, _1, boost::ref(k)));
	}
}

void planes_coefficients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	// Get plane coefficients and indices.
	std::vector<pcl::ModelCoefficients> planesCoefficients;
	std::vector<pcl::PointIndices::Ptr> planesIndices;
	getPlanes(cloud, NUM_PLANES, planesCoefficients, planesIndices);

	// Safe copy of the coefficients to the global variable.
	for(int i = 0; i < k.planes.size(); i++) {
		k.m_coef.lock();
		k.planes[i].coefficients = planesCoefficients[i];
		k.m_coef.unlock();
	}
	k.sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGBA>>(k.sub_channel, 1, boost::bind(calculate_limits, _1, boost::ref(k)));
	ROS_INFO("Calculated coefficients %s", k.id.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "extract_plane");
	nh.reset(new ros::NodeHandle());

	ks = std::vector<kinect>(NUM_CAMS);
	ks[0].init(CAM1, "cam1_link", QUALITY_CAM, NUM_PLANES);
	ks[1].init(CAM2, "cam2_link", QUALITY_CAM, NUM_PLANES);

	for(int i = 0; i < ks.size(); i++) {
		ks[i].sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGBA>>(ks[i].sub_channel, 1, boost::bind(planes_coefficients, _1, boost::ref(ks[i])));
		ks[i].plane_pub = nh->advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(ks[i].pub_plane_channel, 1);
		ks[i].marker_pub = nh->advertise<visualization_msgs::Marker>(ks[i].pub_marker_channel, 1);
	}

	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
   	spinner.spin();
}