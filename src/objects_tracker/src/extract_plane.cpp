// Ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <objects_tracker/Objects.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

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
#include <objects_tracker/segmentation/MultiplePlaneSegmentation.hpp>

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

boost::shared_ptr<ros::NodeHandle> nh;

bool newCloud = false;
boost::mutex m;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

void createObjects(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::vector<pcl::PointIndices>& objectIndices, std::string frame_id, objects_tracker::Objects& obs) {

	// Build ros message.
	obs = objects_tracker::Objects();
	obs.objects.resize(objectIndices.size());
	obs.header.frame_id = frame_id;

	// Recognition.
	#pragma omp parallel for shared(cloud, objectIndices, obs) num_threads(10)
	for(int i = 0; i < obs.objects.size(); i++) {
		
		// Create object point cloud.
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGBA>());
		pcl::PointIndices::ConstPtr inds = boost::make_shared<pcl::PointIndices>(objectIndices[i]);
		extractIndices(cloud, inds, pc);
		pcl::PointIndices subObjectIndices = objectIndices[i];
		subPointCloud(pc, subObjectIndices);

		// Extract minimum and maximum point, position and rotation.
		pcl::PointXYZRGBA min_point, max_point, pos_point;
		Eigen::Matrix3f rotational_matrix;
		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
		feature_extractor.setInputCloud(pc);
		feature_extractor.setIndices(inds);
		feature_extractor.compute();
		feature_extractor.getOBB(min_point, max_point, pos_point, rotational_matrix);

		// Compute pose.
		Eigen::Quaternionf quat(rotational_matrix);
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = frame_id;
		pose.header.stamp = ros::Time();
		pose.pose.orientation.x = quat.x(); pose.pose.orientation.y = quat.y(); pose.pose.orientation.z = quat.z(); pose.pose.orientation.w = quat.w();
		pose.pose.position.x = pos_point.x; pose.pose.position.y = pos_point.y; pose.pose.position.z = pos_point.z;

		// Copy minimum and maximum point.
		geometry_msgs::Point min_pt, max_pt;
		min_pt.x = min_point.x; min_pt.y = min_point.y; min_pt.z = min_point.z;
		max_pt.x = max_point.x; max_pt.y = max_point.y; max_pt.z = max_point.z;

		// Create object.
		objects_tracker::Object ob;
		pcl::toROSMsg(*pc, ob.point_cloud);
		ob.bb.min_pt = min_pt;
		ob.bb.max_pt = max_pt;
		ob.bb.pose = pose;
		ob.indices = subObjectIndices.indices;
		ob.name = "";
		obs.objects[i] = ob;
	}
}

void publishBoundaries(const ros::TimerEvent&, const MultiplePlaneSegmentation &s, const ros::Publisher &marPub, const std::string &frame_id) {

	if (marPub.getNumSubscribers() > 0) {
		std::vector<std::vector<pcl::PointXYZRGBA>> boundaries;
		s.getBoundaries(boundaries);
		
		for(int i = 0; i < boundaries.size(); i++) {
			std::vector< std::vector<double> > positions = std::vector< std::vector<double> >(boundaries[i].size() + 1, std::vector<double>(3,0));
			int j;

			if (boundaries[i].size() == 0) continue;

			// Lines between consecutive points.
			for(j = 0; j < boundaries[i].size(); j++) {
				pcl::PointXYZRGBA p = boundaries[i][j];

				positions[j][0] = p.x;
				positions[j][1] = p.y;
				positions[j][2] = p.z;
			}
			// create a line between last and first point.
			positions[j] = positions[0];

			double width = 0.03;
			std::vector<double> color;
			computeColor(i, boundaries.size(), color);
			double colorArr[] = {color[0], color[1], color[2], 1.0};
			marPub.publish(buildLineMarker(frame_id, i, positions, width, colorArr));
	 	}
	}
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &callbackCloud) {
	m.lock();
	*cloud = *callbackCloud;
	pcl::copyPointCloud(*callbackCloud, *cloud);
	m.unlock();
	newCloud = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "extract_plane");
	nh.reset(new ros::NodeHandle());

	if(argc < 5){
	    std::cout << std::endl;
	    cout << "Not enough arguments provided." << endl;
	    cout << "Usage: ./pose_calibration <cam> <quality> <link> <num_planes>" << endl;
	    return 0;
	}

	// Compute plane direction.
	std::string path = ros::package::getPath("objects_tracker");
	std::string file = path + "/cfg/tf.yaml";

	YAML::Node config;
	try {
		config = YAML::LoadFile(file); // gets the root node
	} catch (YAML::BadFile bf) {
		ROS_ERROR("No configuration file tf.yaml found, tf_calibration node must be run before extracting objects.");
		return 0;
	}

	cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

	std::string cam(argv[1]);
	std::string quality(argv[2]);
	std::string frame_id(argv[3]);
	int nPlanes = atoi(argv[4]);

	std::string subTopic = "/" + cam + "/" + quality + "/PointCloud";
	std::string segTopic = "/" + cam + "/objects";
	std::string marTopic = "/" + cam + "/planes";

	YAML::Node par = config[cam];
	Eigen::Quaternion<float> quat(par["qw"].as<float>(), par["qx"].as<float>(), par["qy"].as<float>(), par["qz"].as<float>());
	Eigen::Vector3f planeOrientation = quat.toRotationMatrix()*Eigen::Vector3f(1,0,0);
	planeOrientation.normalize();

	int nCumulative = 50;

	ros::Subscriber sub = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGBA>>(subTopic, 1, cloudCallback);
	ros::Publisher segPub = nh->advertise<objects_tracker::Objects>(segTopic, 1);
	ros::Publisher marPub = nh->advertise<visualization_msgs::Marker>(marTopic, 100);

	MultiplePlaneSegmentation s(nPlanes, planeOrientation);

	// Compute planes coefficients.
	while(ros::ok()) {
		if(newCloud) {

			// Safe point cloud copy.
			m.lock();
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::copyPointCloud(*cloud, *currentCloud);
			m.unlock();

			s.computePlanes(currentCloud);

			newCloud = false;
			break;
		}
		ros::spinOnce();
	}
	ROS_INFO("Coefficients computed");

	// Compute plane boundaries.
	int i = 0;
	while(ros::ok()) {
		if(newCloud) {

			// Safe point cloud copy.
			m.lock();
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::copyPointCloud(*cloud, *currentCloud);
			m.unlock();

			// Update masks.
			s.updateMasks(currentCloud);

			newCloud = false;
			i++;

			// Compute boundaries and stop iteration when it is necessary.
			if(i == nCumulative) {
				s.computeBoundaries(currentCloud);
				break;
			}
		}
		ros::spinOnce();
	}
	ROS_INFO("Boundaries computed");

	// Timer to publish boundaries.
	ros::Timer timer = nh->createTimer(ros::Duration(10), boost::bind(publishBoundaries, _1, boost::cref(s), boost::cref(marPub), boost::cref(frame_id)));

	// Publish segmented scene.
	while(ros::ok()) {
		if(newCloud and segPub.getNumSubscribers() > 0) {

			// Safe point cloud copy.
			m.lock();
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::copyPointCloud(*cloud, *currentCloud);
			m.unlock();

			newCloud = false;

			// Segment point cloud.
			std::vector<pcl::PointIndices> objectIndices;
			s.segment(cloud, objectIndices);

			objects_tracker::Objects obs;
			createObjects(cloud, objectIndices, frame_id, obs);

			segPub.publish(obs);
		}
		ros::spinOnce();
	}
}