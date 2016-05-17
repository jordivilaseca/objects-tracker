#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>

#include <objects_tracker/Objects.h>
#include <objects_tracker/utilities/ros.hpp>
#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/recognition.hpp>

#include <mutex>
#include <thread>
#include <iostream>
#include <cstring>


#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <Eigen/Geometry>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointIndices::Ptr objectInd(new pcl::PointIndices());
bool newObject = false;
mutex m;

Recogniser::DTYPE dtype = Recogniser::DTYPE::BOTH;
Recogniser r(dtype);

template <typename T>
std::vector<T> computeHeader(const std::vector<T> &list) {
  std::vector<T> header(list.size());

  if(list.size() == 0) return header;

  header[0] = list[0];
  int nelem = 1;
  for(int i = 1; i < list.size(); i++) {
    if(list[i-1] != list[i]) header[nelem++] = list[i];
  }
  header.resize(nelem);

  return header;
}

void publish_object_tf(const ros::TimerEvent&, tf::TransformBroadcaster &br, const Eigen::Vector3f &pos, const Eigen::Vector4f &quat, const std::string &frame_id) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
  transform.setRotation(tf::Quaternion(quat[1], quat[2], quat[3], quat[0]));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, "/object_tf"));
}

void pose_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg, int &i, int max_i, Eigen::Vector4f &quat, Eigen::Vector3f &pos) {
  if(msg->markers.size() > 1) {
    ROS_WARN("Found more than one marker, it cannot happen!");
    return;
  }

  if(msg->markers.size() == 0) return;

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

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, double dist, const pcl::ModelCoefficients &plane, const std::vector<pcl::PointXYZRGBA> &limits, ros::Publisher &pointcloud_pub) {
  pcl::PointIndices::Ptr trainingIndices(new pcl::PointIndices());
  trainingIndices->indices.resize(cloud->points.size());

  Eigen::Vector4f n(plane.values.data());
  int numIndices = 0;

  // Select only the points that are inside the training area.
  for(size_t i = 0; i < cloud->points.size(); i++) {
    // Calculate the distance from the point to the plane normal as the dot product
    // D =(P-A).N/|N|

    if(isnan(cloud->points[i].x)) continue;
    if (isInlier(cloud, i , limits, n)) {
      Eigen::Vector4f pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1);
      float distance = n.dot(pt);

      if (distance > 0.02 and fabsf(distance) <= dist) {
        trainingIndices->indices[numIndices++] = i;
      }
    }
  }

  trainingIndices->indices.resize(numIndices);

  if(trainingIndices->indices.size() == 0) return;

  // Divide training region in clusters and use only the biggest one.
  std::vector<pcl::PointIndices> clusterIndices;
  clustering(cloud, trainingIndices, 0.03, 200, clusterIndices);
  std::vector<pcl::PointIndices>::iterator minClusterIndices;

  if(clusterIndices.size() == 0) return;

  minClusterIndices = std::max_element(std::begin(clusterIndices), std::end(clusterIndices), [](pcl::PointIndices x, pcl::PointIndices y) {  return x.indices.size() < y.indices.size(); });

  pcl::PointIndices::Ptr minIndices = boost::make_shared<pcl::PointIndices>(*minClusterIndices);

  // Create object point cloud.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr obj(new pcl::PointCloud<pcl::PointXYZRGBA>());
  extractIndices(cloud, minIndices, obj);
  subPointCloud(obj, *minIndices);

  // Safe copy.
  m.lock();
  object = obj;
  objectInd = minIndices;
  newObject = true;
  m.unlock();

  pointcloud_pub.publish(*object);
}

void print_help() {
  cout << endl << "Welcome to the objects training node! You have the next options.\n\t[1] Update training set.\n\t[2] Update testing set.\n\t[3] Train model.\n\t[4] Try training.\n\t[5] Compute confusion matrix.\n\t[6] Change descriptor used.\n\t[7] Change Parameters.\n\t[8] Exit\n\nEnter an option: ";
}

void async_read(bool &newOption, std::string &option) {
  std::string current_option;
  while(true) {
    cin >> option;
    newOption = true;
  }
}

void computeConfusionMatrix(const Recogniser &r, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &objectsNames, const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &objects, const std::vector<pcl::PointIndices> &objectsIndices, std::vector<std::vector<int>> &confMat) {
  
  int nelem = trainingHeader.size();

  confMat = std::vector<std::vector<int>>(nelem, std::vector<int>(nelem, 0));

  for(int k = 0; k < objectsNames.size(); k++) {
    // It contains [object, numPhotos]

    std::string goodName = objectsNames[k];
    
    std::string name = r.predict(objects[k], objectsIndices[k]);
  
    // Find position in confusion matrix.
    auto iti = std::find(trainingHeader.begin(), trainingHeader.end(), goodName);
    int i = std::distance(trainingHeader.begin(), iti);

    // Update confusion matrix.
    if(name == goodName) {
      confMat[i][i] += 1;
    } else {
      auto itj = std::find(trainingHeader.begin(), trainingHeader.end(), name);
      int j = std::distance(trainingHeader.begin(), itj);
      confMat[i][j] += 1;
    }
  }
}

void computeMetrics(const std::vector<std::vector<int>> &confMat, float &accur, std::vector<float> &precision, std::vector<float> &recall, std::vector<float> &fmeasure) {
  // Compute accuracy.
  int total = 0;
  int correct = 0;
  for(int i = 0; i < confMat.size(); i++) {
    for(int j = 0; j < confMat[0].size(); j++) {
      if (i == j) correct += confMat[i][j];
      total += confMat[i][j];
    }
  }
  accur = (float) correct / (float) total;

  // Compute precision.
  precision = std::vector<float>(confMat.size());
  for(int i = 0; i < confMat.size(); i++) {
    int truePositive = confMat[i][i];
    int falsePositive = 0;
    for(int j = 0; j < confMat[i].size(); j++) {
      if(i != j) falsePositive += confMat[j][i];
    }
    precision[i] = (float) truePositive/((float) truePositive + (float) falsePositive);
  }

  // Compute recall.
  recall = std::vector<float>(confMat.size());
  for(int i = 0; i < confMat.size(); i++) {
    int truePositive = confMat[i][i];
    int falseNegative = 0;
    for(int j = 0; j < confMat[i].size(); j++) {
      if( i != j) falseNegative += confMat[i][j];
    }
    recall[i] = (float) truePositive/((float) truePositive + (float) falseNegative);
  }

  // Compute F measures.
  fmeasure = std::vector<float>(confMat.size());
  for(int i = 0; i < confMat.size(); i++) {
    fmeasure[i] = 2.0*(precision[i]*recall[i])/(precision[i] + recall[i]);
  }
} 

void readPointClouds(const std::string &path, std::vector<std::string> &objectsNames, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &objects, std::vector<pcl::PointIndices> &objectsIndices) {
  
  objectsNames = std::vector<std::string>();
  objects = std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
  objectsIndices = std::vector<pcl::PointIndices>();

  boost::filesystem::path dir(path);

  // Iterate over every object to calculate the descriptors.
  for(boost::filesystem::directory_iterator obsIt(dir); obsIt != boost::filesystem::directory_iterator(); ++obsIt) {

    // Check if it is a directory.
    if(!boost::filesystem::is_directory(obsIt->status())) continue;

    // Iterate over every representation of the object.
    for(boost::filesystem::directory_iterator obIt(obsIt->path()); obIt != boost::filesystem::directory_iterator(); ++obIt) {
      boost::filesystem::path rep(obIt->path());

      if(rep.extension().string() != ".pcd") {
        cout << "WARN: Not accepted extension for file " << rep.filename() << endl;
        continue; 
      }

      // Load pointcloud.
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::io::loadPCDFile<pcl::PointXYZRGBA>(rep.string(), *cloud);

      // Compute indices.
      pcl::PointIndices indices;
      indices.indices.resize(cloud->points.size());
      int nelems = 0;
      for(int i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZRGBA p = cloud->points[i];
        if(!isnan(p.x)) indices.indices[nelems++] = i;
      }
      indices.indices.resize(nelems);

      objectsNames.push_back(obsIt->path().stem().string());
      objects.push_back(cloud);
      objectsIndices.push_back(indices);
    }
  }
}

void computeObjects(const std::string &objects_path) {
  cout << "Object name: ";

  std::string objectName;
  getline(cin, objectName);

  // Create folder containing the objects pcd.
  std::string object_path = objects_path + "/" + objectName;
  boost::filesystem::path dir(object_path);
  if(!boost::filesystem::create_directories(dir)) {
    cout << "An object with this name exists, the new images will be added" << endl;
  }

  // Find maximum object id.
  int max_id = 0;
  for(boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); ++itr) {
    std::string filename = itr->path().stem().string();

    try{
      int id = atoi(filename.c_str());
      max_id = max(max_id, id);
    } catch (std::exception const & ex) {
      cout << "WARN: " << filename << " is not a valid name, it must be a number. It has been ignored." << endl;
    }            
  }

  cout << "Press enter every moment you want to take a picture, enter 'exit' to stop." << endl; 

  while(true) {
    std::string option;
    getline(cin, option);
    // Stop storing point clouds.
    if(option == "exit") break;

    // Safe copy of the current point cloud.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCopy(new pcl::PointCloud<pcl::PointXYZRGBA>());
    m.lock();
    pcl::copyPointCloud(*object, *objectCopy); 
    newObject = false;
    m.unlock();

    if (objectCopy->points.size() == 0) {
      cout << "There is no object on the tag or it is to small." << endl;
      continue;
    }
    ++max_id;
    pcl::io::savePCDFileASCII (object_path + "/" + to_string(max_id) + ".pcd", *objectCopy);
    cout << "Added " << object_path + "/" + to_string(max_id) + ".pcd" << endl;
  }
}

void tryTraining(const Recogniser &ra) {
  cout << "Press enter every moment you want to take a picture, enter 'exit' to stop." << endl; 

  while(true) {
    std::string option;
    getline(cin, option);

    // Stop trying training.
    if(option == "exit") break;

    // Safe copy of the current point cloud.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCopy(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointIndices::Ptr indicesCopy(new pcl::PointIndices());
    m.lock();
    pcl::copyPointCloud(*object, *objectCopy);
    indicesCopy = objectInd;
    newObject = false;
    m.unlock();

    // Do nothing if the object is empty.
    if (objectCopy->points.size() == 0) {
      cout << "There is no object on the tag or it is to small." << endl;
      continue;
    }

    std::string result = ra.predict(objectCopy, *indicesCopy);
    cout << result << endl;

    /*
    // Compute descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr target(new pcl::PointCloud<pcl::VFHSignature308>());
    computeDescriptors(object, target);
    std::vector<std::vector<float>> target;
    computeDescriptors(objectCopy, *indicesCopy, target);

    //findBestMatch(target, header, kdtree);
    */
  }
}

void parse_option(std::string option, std::string objects_path, std::string frame) {
  if (option.length() != 1) return;

  std::string training_path = objects_path + "/training";
  std::string testing_path = objects_path + "/testing";

  switch(option[0]) {
    case '1':
      computeObjects(training_path);
      break;

    case '2':
      computeObjects(testing_path);
      break;

    case '3':
      {
        // Read objects from disk.
        cout << "Reading objects point clouds..." << flush;
        std::vector<std::string> objectsNames;
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> objects;
        std::vector<pcl::PointIndices> objectsIndices;
        readPointClouds(training_path, objectsNames, objects, objectsIndices);
        cout << "ended" << endl;
  
        // Train model and store it.
        r.addObjects(objects, objectsNames, objectsIndices);
        r.computeModel();
        r.write(training_path);
        break;
      }

    case '4':
    case '5':
      {

        cout << "Reading training model... " << flush;
        r.read(training_path);
        cout << "ended" << endl;

        if(option[0] == '4') {
          tryTraining(r);
        } else {
          std::vector<std::string> objectsNames;
          std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> objects;
          std::vector<pcl::PointIndices> objectsIndices;

          std::cout << "Reading point clouds... " << std::flush;
          readPointClouds(testing_path, objectsNames, objects, objectsIndices);
          std::cout << "ended" << std::endl;

          std::vector<std::string> header = computeHeader(objectsNames);
          std::vector<std::vector<int>> confMat;

          std::cout << "Computing confusion matrix... " << std::flush;
          computeConfusionMatrix(r, header, objectsNames, objects, objectsIndices, confMat);
          std::cout << "ended" << std::endl;

          std::cout << "Computing metrics... " << std::flush;
          float accur;
          std::vector<float> precision;
          std::vector<float> recall;
          std::vector<float> fmeasure;
          computeMetrics(confMat, accur, precision, recall, fmeasure);
          std::cout << "ended" << std::endl;

          cout << "Enter a file name: ";
          std::string name;
          getline(cin, name);
          writeMetrics(confMat, accur, precision, recall, fmeasure, header, objects_path + "/" + name + ".csv");
        }
        break;
      }
    case '6':
      {
        cout << "Available types are:\n\t[1] Colour.\n\t[2] Shape.\n\t[3] Both.\n\n";
        std::string type;
        getline(cin, type);

        if (type.length() != 1) cout << "Answer not correct" << endl;

        switch(type[0]) {
          case '1':
            r.setDescriptor(Recogniser::DTYPE::COLOR);
            break;
          case '2':
            r.setDescriptor(Recogniser::DTYPE::SHAPE);
            break;
          case '3':
            r.setDescriptor(Recogniser::DTYPE::BOTH);
            break;
          default:
            cout << "Answer not correct" << endl;
        }
        break;
      }
    case '7':
     {
      cout << "Current options values:" << endl;
      cout << "\t[1] Clusters per objects = " << r.clustersPerObject << endl;
      cout << "\t[2] Normal estimation distance = " << r.normalEstimationDist << endl;
      cout << "\t[3] Region Growing smoothness angle = " << r.RGAngle << endl;
      cout << "\t[4] Region Growing curvature angle = " << r.RGCurvature << endl;
      cout << "\t[5] Region Growing Number of neighbours = " << r.RGNumNeighbours << endl;
      cout << "\t[6] Region Growing Minimum cluster size = " << r.RGMinClusterSize << endl;
      cout << "Set option " << flush;

      std::string type;
      getline(cin, type);
      cout << "\nSet new value " << flush;

      std::string value;
      getline(cin, value);

      switch(type[0]) {
        case '1':
          r.clustersPerObject = stoi(value);
          break;
        case '2':
          r.normalEstimationDist = stof(value);
          break;
        case '3':
          r.RGAngle = stof(value);
          break;
        case '4':
          r.RGCurvature = stof(value);
          break;
        case '5':
          r.RGNumNeighbours = stoi(value);
          break;
        case '6':
          r.RGMinClusterSize = stoi(value);
          break;
      }
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "objects_training");
  ros::NodeHandle nh;

  if(argc < 4){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./objects_training <marker_size> <frame> <image_topic>" << endl;
    return 0;
  }

  float markerSize = atof(argv[1])/200.0;
  std::string frame = argv[2];
  std::string topic = argv[3];

  // Variables used to calculate the plane coefficients.
  int i = 0;
  int max_i = 25;
  Eigen::Vector4f quat(0,0,0,0);
  Eigen::Vector3f pos(0,0,0);

  ros::Subscriber sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, boost::bind(pose_callback, _1, boost::ref(i), max_i, boost::ref(quat), boost::ref(pos)));
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/objects_training/markers", 50);
  ros::Publisher pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("objects_training/object", 5);

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
  std::vector<pcl::PointXYZRGBA> limits(4);
  std::vector<std::vector<double>> xyDir = {{1,1}, {1,-1}, {-1,-1}, {-1,1}};
  for(int i = 0; i < xyDir.size(); i++) {
    double xDir = xyDir[i][0];
    double yDir = xyDir[i][1];

    pcl::PointXYZRGBA p;
    p.x = pos[0] + markerSize*(xDir*x[0] + yDir*y[0]);
    p.y = pos[1] + markerSize*(xDir*x[1] + yDir*y[1]);
    p.z = (p.x*plane.values[0] + p.y*plane.values[1] + plane.values[3])/-plane.values[2];

    limits[i] = p;
  }

  // Publish object tf.
  tf::TransformBroadcaster br;
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(publish_object_tf, _1, boost::ref(br), boost::cref(pos), boost::cref(quat), boost::cref(frame)));

  //publish bounding box
  double markerPos0[] = {pos[0] + z[0]*markerSize, pos[1] + z[1]*markerSize, pos[2] + z[2]*markerSize};
  double scale0[] = {2.0*markerSize, 2.0*markerSize, 2.0*markerSize};
  double color0[] = {0, 0, 1, 0.1};
  double orien0[] = {(double) q.x(), (double) q.y(), (double) q.z(), (double) q.w()};
  marker_pub.publish(buildMarker(frame, 0, visualization_msgs::Marker::CUBE, markerPos0, scale0, color0, orien0));
  ROS_INFO("Published training area in /objects_training/markers. Only the points inside it will be used for training");

  // Publish axis.
  std::vector< std::vector<double> > markerPos1 = {{pos[0], pos[1], pos[2]}, {pos[0]+x[0], pos[1]+x[1], pos[2]+x[2]}};
  double color[] = {1, 0, 0, 1};
  marker_pub.publish(buildLineMarker(frame, 1, markerPos1, 0.02, color));

  std::vector< std::vector<double> > markerPos2 = {{pos[0], pos[1], pos[2]}, {pos[0]+y[0], pos[1]+y[1], pos[2]+y[2]}};
  double color2[] = {0, 1, 0, 1};
  marker_pub.publish(buildLineMarker(frame, 2, markerPos2, 0.02, color2));

  std::vector< std::vector<double> > markerPos3 = {{pos[0], pos[1], pos[2]}, {pos[0]+z[0], pos[1]+z[1], pos[2]+z[2]}};
  double color3[] = {0, 0, 1, 1};
  marker_pub.publish(buildLineMarker(frame, 3, markerPos3, 0.01, color3));
  ROS_INFO("Published axis in /objects_training/markers");

  ROS_INFO("Ready to learn a new object!");
  sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGBA>>(topic, 1, boost::bind(pointcloud_callback, _1, markerSize*2.0, boost::ref(plane), boost::ref(limits), boost::ref(pointcloud_pub)));

  std::string option;
  
  std::string path = ros::package::getPath("objects_tracker");

  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();

  // Treatment of objects and options.
  while(true) {
    print_help();
    getline(cin, option);

    if (option == "8") break;

    parse_option(option, path, frame);
    cout << "Enter an option: ";
  }
  spinner.stop();
}