#include <ros/ros.h>
#include <ros/package.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <objects_tracker/Objects.h>
#include <objects_tracker/utilities/ros.hpp>
#include <objects_tracker/utilities/utilities.hpp>

#include <mutex>

#include <Eigen/Geometry>

using namespace std;

void pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg, int &i, Eigen::Vector3f &acum_normal, float &acum_d) {
  if(msg->markers.size() > 1) {
    ROS_WARN("Found more than one marker, it cannot happen!");
    return;
  }

  if(i == 0) ROS_INFO("Tag Found!");

  geometry_msgs::Pose pose_msg = msg->markers[0].pose.pose;
  Eigen::Quaternion<float> quat(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
  Eigen::Vector3f normal = quat.toRotationMatrix()*Eigen::Vector3f(0,0,1);
  normal.normalize();
  float d = -(normal[0]*pose_msg.position.x + normal[1]*pose_msg.position.y + normal[2]*pose_msg.position.z);

  acum_normal += Eigen::Vector3f(normal[0], normal[1], normal[2]);
  acum_d += d;

  i++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_training");
  ros::NodeHandle nh;

  // Variables used to calculate the plane coefficients.
  int i = 0;
  int max_i = 25;
  Eigen::Vector3f acum_normal;
  float acum_d;

  ros::Subscriber sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, boost::bind(pose_callback, _1, boost::ref(i), boost::ref(acum_normal), boost::ref(acum_d)));

  ROS_INFO("Looking for a tag...");
  while(i < max_i) {
    ros::spinOnce();
  }
  acum_normal /= (float) max_i; acum_d /= max_i;
  acum_normal.normalize();
  pcl::ModelCoefficients plane;
  plane.values = std::vector<float>{acum_normal[0], acum_normal[1], acum_normal[2], acum_d};
  cout << acum_normal << " d " << acum_d << endl << plane << endl; 
  //sub.shutdown();
  ROS_INFO("Ready to learn a new object!");

  ros::spin();
}
