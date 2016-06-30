//ar_track_alvar
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace std;

struct tf {
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;

  tf() : x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(0) {}
};

tf t;
int iter = 0;
int total_iter = 25;

void pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if (msg->markers.size() > 1) {
    ROS_WARN("Found more than one marker, nothing done.");
  } else {
    geometry_msgs::Pose pose = msg->markers[0].pose.pose;
    t.x += pose.position.x;
    t.y += pose.position.y;
    t.z += pose.position.z;
    t.qx += pose.orientation.x;
    t.qy += pose.orientation.y;
    t.qz += pose.orientation.z;
    t.qw += pose.orientation.w;
    iter++;
    ROS_INFO("Received pose %i from %i", iter, total_iter);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_calibration");
  ros::NodeHandle nh;

  if(argc < 3){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./pose_calibration <cam> <quality>" << endl;
    return 0;
  }

  std::string path = ros::package::getPath("objects_tracker");
  std::string file = path + "/cfg/tf.yaml";

  YAML::Node config;
  try {
    config = YAML::LoadFile(file); // gets the root node
  } catch (YAML::BadFile bf) {
    ROS_WARN("No configuration file found, a new one will be created");
    config = YAML::Load("");
  }

  ros::Subscriber p = nh.subscribe("/ar_pose_marker", 10, pose_callback);

  while(ros::ok()) {
    if(iter >= total_iter) {
      p.shutdown();
      ROS_INFO("Reached limit of poses received");

      // Computing mean.
      t.x /= iter;
      t.y /= iter;
      t.z /= iter;
      t.qx /= iter;
      t.qy /= iter;
      t.qz /= iter;
      t.qw /= iter;

      std::vector<float> pos = {t.x, t.y, t.z};
      std::vector<float> quat = {t.qx, t.qy, t.qz, t.qw};
      config[argv[1]]["pos"] = pos;
      config[argv[1]]["quat"] = quat;
      config[argv[1]]["parent_frame"] = "cams_pose";

      std::ofstream fout(file);
      fout << config << endl;
      fout.close();
      ROS_INFO("Configuration written at %s", file.c_str());
      ROS_INFO("Pose calibrated, please press ctr+C to end the calibration");
      ros::shutdown();
    }
    ros::spinOnce();
  }
}
