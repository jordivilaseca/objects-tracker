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

void pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg, int &i, int max_i, Eigen::Vector4f &quat, Eigen::Vector3f &pos) {
  if(msg->markers.size() > 1) {
    ROS_WARN("Found more than one marker, it cannot happen!");
    return;
  }

  if(i == 0) ROS_INFO("Tag Found! The average of %i lectures of the tag will be calculated", max_i);
  if(i >= max_i) return;

  // Accumulate quaternion.
  geometry_msgs::Pose pose_msg = msg->markers[0].pose.pose;
  Eigen::Vector4f new_quat(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
  quat += new_quat;

  // Accumulate position.
  Eigen::Vector3f new_pos(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  pos += new_pos;

  i++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_training");
  ros::NodeHandle nh;

  if(argc < 3){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./objects_training <marker_size> <frame>" << endl;
    return 0;
  }

  float markerSize = atof(argv[1])/200.0;
  std::string frame = argv[2];

  // Variables used to calculate the plane coefficients.
  int i = 0;
  int max_i = 25;
  Eigen::Vector4f quat(0,0,0,0);
  Eigen::Vector3f pos(0,0,0);

  ros::Subscriber sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, boost::bind(pose_callback, _1, boost::ref(i), max_i, boost::ref(quat), boost::ref(pos)));
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/objects_training/markers", 50);

  ROS_INFO("Looking for a tag...");
  while(i < max_i) {
    ros::spinOnce();
  }
  sub.shutdown();
  ROS_INFO("Finished the lecture of tags");

  // Calculate the average quaternion.
  quat /= (float) max_i;
  quat.normalize();

  // Calculate the average position.
  pos /= (float) max_i;

  // Calculate new axis orientations.
  Eigen::Quaternion<float> q(quat[0], quat[1], quat[2], quat[3]);
  Eigen::Vector3f x = q.toRotationMatrix()*Eigen::Vector3f(1,0,0); x.normalize();
  Eigen::Vector3f y = q.toRotationMatrix()*Eigen::Vector3f(0,1,0); y.normalize();
  Eigen::Vector3f z = q.toRotationMatrix()*Eigen::Vector3f(0,0,1); z.normalize();

  // Calculate plane coefficients.
  pcl::ModelCoefficients plane;
  float d = -(z[0]*pos[0] + z[1]*pos[1] + z[2]*pos[2]);
  plane.values = std::vector<float>{z[0], z[1], z[2], d};

  // Calculate limits.
  std::vector<std::vector<double>> limits(4, std::vector<double>(3));
  std::vector<std::vector<double>> xyDir = {{1,1}, {1,-1}, {-1,-1}, {-1,1}};
  for(int i = 0; i < xyDir.size(); i++) {
    double xDir = xyDir[i][0];
    double yDir = xyDir[i][1];

    double xPoint = pos[0] + markerSize*(xDir*x[0] + yDir*y[0]);
    double yPoint = pos[1] + markerSize*(xDir*x[1] + yDir*y[1]);
    double zPoint = (xPoint*plane.values[0] + yPoint*plane.values[1] + plane.values[3])/-plane.values[2];

    limits[i] = {xPoint, yPoint, zPoint};
  }

  //publish bounding box
  double markerPos0[] = {pos[0] + z[0]*markerSize, pos[1] + z[1]*markerSize, pos[2] + z[2]*markerSize};
  double scale0[] = {2.0*markerSize, 2.0*markerSize, 2.0*markerSize};
  double color0[] = {0, 0, 1, 0.1};
  double orien0[] = {(double) q.x(), (double) q.y(), (double) q.z(), (double) q.w()};
  pub.publish(buildMarker(frame, 0, visualization_msgs::Marker::CUBE, markerPos0, scale0, color0, orien0));
  ROS_INFO("Published training area in /objects_training/markers. Only the points inside it will be used for training");

  // Publish axis.
  std::vector< std::vector<double> > markerPos1 = {{pos[0], pos[1], pos[2]}, {pos[0]+x[0], pos[1]+x[1], pos[2]+x[2]}};
  double color[] = {1, 0, 0, 1};
  pub.publish(buildLineMarker(frame, 1, markerPos1, 0.02, color));

  std::vector< std::vector<double> > markerPos2 = {{pos[0], pos[1], pos[2]}, {pos[0]+y[0], pos[1]+y[1], pos[2]+y[2]}};
  double color2[] = {0, 1, 0, 1};
  pub.publish(buildLineMarker(frame, 2, markerPos2, 0.02, color2));

  std::vector< std::vector<double> > markerPos3 = {{pos[0], pos[1], pos[2]}, {pos[0]+z[0], pos[1]+z[1], pos[2]+z[2]}};
  double color3[] = {0, 0, 1, 1};
  pub.publish(buildLineMarker(frame, 3, markerPos3, 0.01, color3));
  ROS_INFO("Published axis in /objects_training/markers");


  ROS_INFO("Ready to learn a new object!");

  ros::spin();
}
