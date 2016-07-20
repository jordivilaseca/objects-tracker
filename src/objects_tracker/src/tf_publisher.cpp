//ar_track_alvar
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include "yaml-cpp/yaml.h"

using namespace std;

/*! \file */

YAML::Node config;

/**
 * @brief It publishes the poses periodically.
 * 
 * @param br Transform broadcaster to send the tfs.
 */
void publish_poses(const ros::TimerEvent&, tf::TransformBroadcaster &br) {
  for (auto itCam = config.begin(); itCam != config.end(); ++itCam) {
    YAML::Node cam = itCam->first;
    YAML::Node par = itCam->second;

    tf::Transform transform;
    std::vector<float> pos = par["pos"].as<std::vector<float>>();
    std::vector<float> quat = par["quat"].as<std::vector<float>>();

    transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
    transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), par["parent_frame"].as<std::string>(), par["frame"].as<std::string>()));
  }
}

/**
 * @brief Node in charge of send periodically all the available relative poses between cameras. 
 * @details It makes use of the information gathered using the 'tf_calibration' node and publishes it.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;

  std::string path = ros::package::getPath("objects_tracker");
  std::string file = path + "/cfg/tf.yaml";

  ros::Timer timer;
  try {
    config = YAML::LoadFile(file); // gets the root node
    timer = nh.createTimer(ros::Duration(1), boost::bind(publish_poses, _1, boost::ref(br)));
  } catch (YAML::BadFile) {
    ROS_ERROR("TF not found, searched at %s. The node is going to stop.", file.c_str());
    return 0;
  }

  ros::spin();
}
