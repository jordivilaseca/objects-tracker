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

using namespace std;

YAML::Node config;

void publish(const objects_tracker::Objects::ConstPtr &obs, std::string frame_id, ros::Publisher &pub_bb, ros::Publisher &pub_pc) {
  if (pub_bb.getNumSubscribers() > 0) {

    // Remove old markers.
    visualization_msgs::Marker deleteAll;
    deleteAll.action = 3;
    pub_bb.publish(deleteAll);

    // Publish bounding box as a square marker with small alpha.
    for (int i = 0; i < obs->objects.size(); i++) {
      objects_tracker::BoundingBox bb = obs->objects[i].bb;

      std::vector<double> col(4,0.0);
      computeColor(i, obs->objects.size(), col);
      double color[] = {col[0], col[1], col[2], 0.5};

      geometry_msgs::PoseStamped pose = obs->objects[i].bb.pose;
      double pos[] = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
      double scale[] = {bb.max_pt.x - bb.min_pt.x, bb.max_pt.y - bb.min_pt.y, bb.max_pt.z - bb.min_pt.z};
      double orien[] = {pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w};
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

  if(argc < 2){
      std::cout << std::endl;
      cout << "Not enough arguments provided." << endl;
      cout << "Usage: ./objects_recognition <cam> <link>" << endl;
      return 0;
  }

  std::string cam(argv[1]);
  std::string frame_id(argv[1]);
  std::string topic = "/" + cam + "/objects";
  std::string namedTopic = "/" + cam + "/namedObjects";

  ros::Publisher pub_bb = nh.advertise<visualization_msgs::Marker>(topic + "/boundingbox", 50);
  ros::Publisher pub_nbb = nh.advertise<visualization_msgs::Marker>(namedTopic + "/boundingbox", 50);
  ros::Publisher pub_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(topic + "/pointcloud", 50);
  ros::Subscriber sub_bb = nh.subscribe<objects_tracker::Objects>(topic, 1, boost::bind(publish, _1, frame_id, boost::ref(pub_bb), boost::ref(pub_pc)));
  ros::Subscriber sub_nbb = nh.subscribe<objects_tracker::Objects>(namedTopic, 1, boost::bind(publish, _1, frame_id, boost::ref(pub_nbb), boost::ref(pub_pc)));

  ros::spin();
}
