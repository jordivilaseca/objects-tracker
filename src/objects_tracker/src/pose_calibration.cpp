//ar_track_alvar
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>
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
  ros::init(argc, argv, "pose_calibration");
  ros::NodeHandle nh;

  YAML::Node config = YAML::Load("{}"); // gets the root node

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

      config["x"] = t.x;
      config["y"] = t.y;
      config["z"] = t.z;
      config["qx"] = t.qx;
      config["qy"] = t.qy;
      config["qz"] = t.qz;
      config["qw"] = t.qw;

      std::ofstream fout("/home/inhands-user2/catkin_ws/config.yaml");
      fout << config << endl;
      fout.close();
      ROS_INFO("Pose calibrated, please press ctr+C to end the calibration");
      ros::shutdown();
    }
    ros::spinOnce();
  }
}
