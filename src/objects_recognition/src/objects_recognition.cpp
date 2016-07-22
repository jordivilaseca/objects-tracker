#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <objects_tracker/Objects.h>

#include <objects_tracker/utilities/ros.hpp>
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/recognition/Recognition.hpp>

using namespace std;

/*! \file */

/**
 * @brief It recognises and publishes the information of different objects.
 * 
 * @param obs Message containing different objects point cloud.
 * @param r The recogniser previously trained.
 * @param pub Publisher to send the extracted information.
 */
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

/**
 * @brief Node in charge of making the recognition of the objects.
 * 
 * It reads the recogniser previously trained using the 'training' node and starts recognising the objects published
 * on the /<cam>/objects, where <cam> is the cam identifier passed as parameter and publishes the results in the
 * topic /<cam>/namedObjects. The only thing that this node does is to complement the input message adding the 
 * identifier of the object.
 * 
 * @param cam cam identifier.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_recognition");
  ros::NodeHandle nh;

  Recogniser r = Recogniser(Recogniser::DTYPE::BOTH);
  r.read(ros::package::getPath("objects_tracker") + "/training");

  if(argc < 2){
      std::cout << std::endl;
      cout << "Not enough arguments provided." << endl;
      cout << "Usage: ./objects_recognition <cam>" << endl;
      return 0;
  }

  std::string cam(argv[1]);
  std::string subTopic = "/" + cam + "/objects";
  std::string pubTopic = "/" + cam + "/namedObjects";

  ros::Publisher pub = nh.advertise<objects_tracker::Objects>(pubTopic, 1);
  ros::Subscriber sub = nh.subscribe<objects_tracker::Objects>(subTopic, 1, boost::bind(make_recognition, _1, boost::cref(r), boost::cref(pub)));

  ros::spin();
}
