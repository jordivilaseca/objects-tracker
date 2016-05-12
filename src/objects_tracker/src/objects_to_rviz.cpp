#include <ros/ros.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <objects_tracker/Objects.h>
#include <visualization_msgs/Marker.h>

#include <objects_tracker/utilities/ros.hpp>
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/bridge.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

YAML::Node config;

void publish(const objects_tracker::Objects::ConstPtr &obs, std::string frame_id, ros::Publisher &pub_bb, ros::Publisher &pub_pc) {
  if (pub_bb.getNumSubscribers() > 0) {
    // Publish bounding box as a square marker with small alpha.
    for (int i = 0; i < obs->objects.size(); i++) {
      objects_tracker::BoundingBox bb = obs->objects[i].bb;

      std::vector<double> col(4,0.0);
      computeColor(i, obs->objects.size(), col);
      double color[] = {col[0], col[1], col[2], 0.5};

      geometry_msgs::Pose pose = obs->objects[i].bb.pose;
      double pos[] = {pose.position.x, pose.position.y, pose.position.z};
      double scale[] = {bb.max_pt.x - bb.min_pt.x, bb.max_pt.y - bb.min_pt.y, bb.max_pt.z - bb.min_pt.z};
      double orien[] = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
      pub_bb.publish(buildMarker(frame_id, 2*i, visualization_msgs::Marker::CUBE, pos, scale, color, orien));
      if(obs->objects[i].name != "") {
        pub_bb.publish(buildText(frame_id, 2*i+1, pos, 0.05, obs->objects[i].name));
      }
    }
  }
  if (pub_pc.getNumSubscribers() > 0) {

    // Publish pointcloud containing all the objects.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for (int i = 0; i < obs->objects.size(); i++) {
      pcl::PointCloud<pcl::PointXYZRGBA> aux = pcl::PointCloud<pcl::PointXYZRGBA>();
      pcl::fromROSMsg(obs->objects[i].point_cloud, aux);
      
      *cloud += pcl::PointCloud<pcl::PointXYZRGBA>(aux, obs->objects[i].indices);
    }
    cloud->header.frame_id = frame_id;
    pub_pc.publish(cloud);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_to_rviz");
  ros::NodeHandle nh;

  std::string path = ros::package::getPath("objects_tracker");
  std::string file = path + "/cfg/tf.yaml";

  YAML::Node config;
  try {
    config = YAML::LoadFile(file); // gets the root node
  } catch (YAML::BadFile bf) {
    ROS_ERROR("No configuration file found, it was searched at %s", file.c_str());
    return 0;
  }

  std::vector<ros::Subscriber> subs(config.size()*config.size());
  std::vector<ros::Publisher> pubs_bb(config.size()*config.size());
  std::vector<ros::Publisher> pubs_pc(config.size());

  int i = 0;
  for (auto itCam = config.begin(); itCam != config.end(); ++itCam, ++i) {
    YAML::Node cam = itCam->first;
    YAML::Node par = itCam->second;
    std::string topic = "/" + cam.as<string>() + "/objects";
    std::string namedTopic = "/" + cam.as<string>() + "/namedObjects";

    pubs_bb[2*i] = nh.advertise<visualization_msgs::Marker>(topic + "/boundingbox", 50);
    pubs_bb[2*i+1] = nh.advertise<visualization_msgs::Marker>(namedTopic + "/boundingbox", 50);
    pubs_pc[i] = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topic + "/pointcloud", 50);
    subs[i*2] = nh.subscribe<objects_tracker::Objects>(topic, 1, boost::bind(publish, _1, cam.as<string>() + "_link", boost::ref(pubs_bb[2*i]), boost::ref(pubs_pc[i])));
    subs[i*2+1] = nh.subscribe<objects_tracker::Objects>(namedTopic, 1, boost::bind(publish, _1, cam.as<string>() + "_link", boost::ref(pubs_bb[2*i+1]), boost::ref(pubs_pc[i])));
  }

  ros::spin();
}
