// Ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <objects_tracker/Objects.h>
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
		std::vector<pcl::PointXYZRGBA> limits;
		std::vector<visualization_msgs::Marker> markers;
	};
	int nplanes;
	int iter;

	std::string id;
	std::string frame_id;
	std::string pub_marker_channel;
	std::string pub_plane_channel;
	std::string pub_objects_channel;
	std::string sub_channel;

	ros::Publisher plane_pub;
	ros::Publisher objects_pub;
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
		pub_objects_channel = "/" + id + "/objects";
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
	clustering(cloud, inputIndices, 0.005, 10000, clusterIndices);

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
}

void remove_planes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	if (k.objects_pub.getNumSubscribers() > 0 or k.plane_pub.getNumSubscribers() > 0) {
		// Cloud containing the points without the planes.
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));
		
		long long init = getTime();

		// -1 -> part of a plane, 0 -> not part of an object, 1 -> part of an object.
		std::vector<char> mask = std::vector<char>(cloud->points.size(), 0);

		for(int i = 0; i < k.nplanes; i++) {

			// Safe load of the coefficients of the global variable.
			k.m_coef.lock();
			pcl::ModelCoefficients planeCoef = k.planes[i].coefficients;
			Eigen::Vector4f coef = Eigen::Vector4f(planeCoef.values.data());
			k.m_coef.unlock();

			// Safe load of the limits of the global variable.
			k.m_limits.lock();
			std::vector<pcl::PointXYZRGBA> planeLimits = k.planes[i].limits;
			k.m_limits.unlock();

			//#pragma omp parallel for firstprivate(threshold, coef) shared(cloud, inliers, nr_p) num_threads(3)
			for(size_t j = 0; j < cloud->points.size(); j++) {
				// Calculate the distance from the point to the plane normal as the dot product
				// D =(P-A).N/|N|

				// If the x value of the pointcloud or it is marked as a point in a plane it is not needed to
				// make further calculations, we don't want this point.
				if(isnan(cloud->points[j].x) or mask[j] == -1) continue;

				if (isInlier(cloud, j , planeLimits, coef)) {
					Eigen::Vector4f pt(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, 1);
					float distance = coef.dot(pt);

					if (fabsf(distance) <= 0.02) {
						// If the point is at a distance less than X, then the point is in the plane, we mark it properly.
						mask[j] = -1;
					} else if (mask[j] == 0 and distance < 0.0){
						// The point is not marked as being part of an object nor plane, if it is above it we mark it as object.
						mask[j] = 1;
					}
				}
			}
		}

		// Parse inliers.
		pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
		inliers->indices.resize(cloud->points.size());
		int nr_p = 0;
		for(int i = 0; i < mask.size(); i++) {
			if(mask[i] == 1) inliers->indices[nr_p++] = i;
		}
		inliers->indices.resize(nr_p);

		// Clustering
		std::vector<pcl::PointIndices> clusterIndices;
		clustering(cloud, inliers, 0.03, 200, clusterIndices);

		// Build ros message.
		objects_tracker::Objects obs;
		obs.objects.resize(clusterIndices.size());
		obs.header.frame_id = k.frame_id;

		#pragma omp parallel for shared(cloud, clusterIndices, obs) num_threads(10)
		for(int i = 0; i < obs.objects.size(); i++) {
			objects_tracker::Object ob;

			// Create object point cloud.
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::PointIndices::ConstPtr inds = boost::make_shared<pcl::PointIndices>(clusterIndices[i]);
			extractIndices(cloud, inds, pc);
			subPointCloud(pc, clusterIndices[i]);

			pcl::toROSMsg(*pc, ob.point_cloud);

			// Find bounding box and mass center.
			pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
			Eigen::Vector3f mass_center;
			pcl::PointXYZRGBA min_point, max_point, pos_point;
			Eigen::Matrix3f rotational_matrix;
			geometry_msgs::Point min_pt, max_pt;
			geometry_msgs::Pose pose;

			feature_extractor.setInputCloud(pc);
			feature_extractor.setIndices(inds);
			feature_extractor.compute();
			feature_extractor.getOBB (min_point, max_point, pos_point, rotational_matrix);
			//feature_extractor.getAABB(min_AABB, max_AABB);
			//feature_extractor.getMassCenter(mass_center);

			//pcl::getMinMax3D(*pc, min_AABB, max_AABB);
			Eigen::Quaternionf quat (rotational_matrix);
			pose.orientation.x = quat.x(); pose.orientation.y = quat.y(); pose.orientation.z = quat.z(); pose.orientation.w = quat.w();
			min_pt.x = min_point.x; min_pt.y = min_point.y; min_pt.z = min_point.z;
			max_pt.x = max_point.x; max_pt.y = max_point.y; max_pt.z = max_point.z;
			pose.position.x = pos_point.x; pose.position.y = pos_point.y; pose.position.z = pos_point.z;
			//mass_pt.x = mass_center[0]; mass_pt.y = mass_center[1]; mass_pt.z = mass_center[2];

			//ob.mass_center = mass_pt;
			ob.bb.min_pt = min_pt;;
			ob.bb.max_pt = max_pt;;
			ob.bb.pose = pose;;

			obs.objects[i] = ob;
		}
		//colourPointCloud(remainingCloud, clusterIndices);

		total_time += getTime()-init;
		times++;
		if (times == MEAN_ELEM) {
			ROS_INFO("Extracted plane points from point cloud %i times, mean time %lu", MEAN_ELEM, total_time/MEAN_ELEM);
			times = 0;
			total_time = 0;
		}

		remainingCloud->header.frame_id = k.frame_id;
		k.plane_pub.publish(remainingCloud);
		k.objects_pub.publish(obs);
	}
}

void initialize_limits(kinect &k, int size) {
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
		pcl::PointIndices planeLimits;
		pcl::PointIndices::ConstPtr planeIndicesConstPtr = boost::make_shared<pcl::PointIndices>(k.planes[i].mask_points);
		int nplanes = getPlaneLimits(cloud, planeIndicesConstPtr, k.planes[i].coefficients, planeLimits);

		std::vector<pcl::PointXYZRGBA> limits = std::vector<pcl::PointXYZRGBA>(planeLimits.indices.size());
		for(int j = 0; j < limits.size(); j++) {
			limits[j] = cloud->points[planeLimits.indices[j]];
		}

		// Safe copy of the limits.
		k.m_limits.lock();
		k.planes[i].limits = limits;
		k.m_limits.unlock();
	}
	ROS_INFO("Calculated limits %s", k.id.c_str());
}

void build_limit_markers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	for(int i = 0; i < k.nplanes; i++) {
		std::vector< std::vector<double> > positions = std::vector< std::vector<double> >(k.planes[i].limits.size() + 1, std::vector<double>(3,0));
		int j;

		// Lines between consecutive points.
		for(j = 0; j < k.planes[i].limits.size(); j++) {
			pcl::PointXYZRGBA p = k.planes[i].limits[j];

			positions[j][0] = p.x;
			positions[j][1] = p.y;
			positions[j][2] = p.z;
		}
		// create a line between last and first point.
		positions[j] = positions[0];

		double width = 0.03;
		std::vector<double> color;
		computeColor(i, k.nplanes, color);
		double colorArr[] = {color[0], color[1], color[2], 255};
		k.planes[i].markers.push_back(buildLineMarker(k.frame_id, i, positions, width, colorArr));
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
		build_limit_markers(cloud, k);
		// Publish markers.
		for(int i = 0; i < k.planes.size(); i++) {
			k.marker_pub.publish(k.planes[i].markers[0]);
		}
		k.existsLimits = true;
		k.sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGBA> >(k.sub_channel, 1, boost::bind(remove_planes, _1, boost::ref(k)));
	}
}

void planes_coefficients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, kinect &k) {
	// Get plane coefficients and indices.
	std::vector<pcl::ModelCoefficients> planesCoefficients;
	std::vector<pcl::PointIndices::Ptr> planesIndices;

	long long init = getTime();
	getPlanes(cloud, NUM_PLANES, planesCoefficients, planesIndices);
	ROS_INFO("Calculated plane coefficients %s, total time %llu", k.id.c_str(), (getTime() - init));

	// Safe copy of the coefficients to the global variable.
	for(int i = 0; i < k.planes.size(); i++) {
		k.m_coef.lock();
		k.planes[i].coefficients = planesCoefficients[i];
		k.m_coef.unlock();
	}
	k.sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGBA>>(k.sub_channel, 1, boost::bind(calculate_limits, _1, boost::ref(k)));
}

void publish_markers(const ros::TimerEvent&) {
	for(int i = 0; i < ks.size(); i++) {
		for (int j = 0; j < ks[i].nplanes; j++) {
			for(int l = 0; l < ks[i].planes[j].markers.size(); l++) {
				ks[i].marker_pub.publish(ks[i].planes[j].markers[l]);
			}
		}
	}
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
		ks[i].objects_pub = nh->advertise<objects_tracker::Objects>(ks[i].pub_objects_channel, 1);
	}

	ros::Timer timer = nh->createTimer(ros::Duration(10), publish_markers);

	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
   	spinner.spin();
}