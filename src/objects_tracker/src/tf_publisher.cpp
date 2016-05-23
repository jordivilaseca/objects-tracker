//ar_track_alvar
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include "yaml-cpp/yaml.h"

using namespace std;

YAML::Node config;

void publish_poses(const ros::TimerEvent&, tf::TransformBroadcaster &br) {
  for (auto itCam = config.begin(); itCam != config.end(); ++itCam) {
    YAML::Node cam = itCam->first;
    YAML::Node par = itCam->second;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(par["x"].as<float>(), par["y"].as<float>(), par["z"].as<float>()));
    transform.setRotation(tf::Quaternion(par["qx"].as<float>(), par["qy"].as<float>(), par["qz"].as<float>(), par["qw"].as<float>()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cam.as<string>() + "_link", cam.as<string>() + "/pose"));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;

  std::string path = ros::package::getPath("objects_tracker");
  std::string file = path + "/cfg/tf.yaml";

  try {
    config = YAML::LoadFile(file); // gets the root node
    ros::Timer timer = nh.createTimer(ros::Duration(5), boost::bind(publish_poses, _1, boost::ref(br)));
  } catch (YAML::BadFile) {
    ROS_ERROR("TF not found, searched at %s. The node is going to stop.", file.c_str());
    return 0;
  }

  ros::spin();
}
