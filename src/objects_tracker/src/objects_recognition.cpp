#include <ros/ros.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <objects_tracker/Objects.h>

#include <objects_tracker/utilities/ros.hpp>
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/recognition.hpp>

using namespace std;

YAML::Node config;

void make_recognition(const objects_tracker::Objects::ConstPtr &obs, const Recogniser &r, const ros::Publisher &pub) {
  if (pub.getNumSubscribers() > 0) {
    long long init = getTime();
    objects_tracker::Objects namedObs = *obs;

    // Publish bounding box as a square marker with small alpha.
    #pragma omp parallel for shared(namedObs) num_threads(10)
    for (int i = 0; i < namedObs.objects.size(); i++) {

      // Predict object.
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr oCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::fromROSMsg(namedObs.objects[i].point_cloud, *oCloud);
      pcl::PointIndices indices;
      indices.indices = namedObs.objects[i].indices;

      namedObs.objects[i].name = r.predict(oCloud, indices);
    }
    pub.publish(namedObs);
    cout << getTime() - init << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_recognition");
  ros::NodeHandle nh;

  std::string path = ros::package::getPath("objects_tracker");
  std::string file = path + "/cfg/cams.yaml";

  Recogniser r = Recogniser(Recogniser::DTYPE::BOTH);
  r.read(ros::package::getPath("objects_tracker") + "/training");

  YAML::Node config;
  try {
    config = YAML::LoadFile(file); // gets the root node
  } catch (YAML::BadFile bf) {
    ROS_ERROR("No configuration file found, it was searched at %s", file.c_str());
    return 0;
  }

  std::cout << "config.size() = " << config.size() << std::endl; 
  std::cout << config[0] << std::endl;
  std::vector<ros::Subscriber> subs(config.size());
  std::vector<ros::Publisher> pubs(config.size());

  int i = 0;
  for (auto itCam = config.begin(); itCam != config.end(); ++itCam, ++i) {
    std::string cam = itCam->as<std::string>();
    std::cout << cam << " " << std::endl;
    std::string subTopic = "/" + cam + "/objects";
    std::string pubTopic = "/" + cam + "/namedObjects";

    pubs[i] = nh.advertise<objects_tracker::Objects>(pubTopic, 1);
    subs[i] = nh.subscribe<objects_tracker::Objects>(subTopic, 1, boost::bind(make_recognition, _1, boost::cref(r), boost::cref(pubs[i])));
  }

  ros::spin();
}
