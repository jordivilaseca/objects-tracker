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
#include <objects_tracker/recognition/Recognition.hpp>

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

/**
 * @brief Returns the different elements of vector. All the elements of a type must be consecutive.
 * 
 * @param list vector containing all the elements
 * @tparam T It could be used any type that has implemented comparison function.
 * @return All the distinct elements that contains the input vector
 */
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

/**
 * @brief Function called by a ROS Timer. It publishes a transformation from 'frame_id' to 'object_tf'.
 * 
 * @param t Timer event.
 * @param br Transform broadcaster to send the transform.
 * @param pos Position of 'object_tf'.
 * @param quat Quaternion rotation of 'object_tf'.
 * @param frame_id Initial frame of the transformation.
 */
void publish_object_tf(const ros::TimerEvent& t, tf::TransformBroadcaster &br, const Eigen::Vector3f &pos, const Eigen::Vector4f &quat, const std::string &frame_id) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));
  transform.setRotation(tf::Quaternion(quat[1], quat[2], quat[3], quat[0]));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, "/object_tf"));
}

/**
 * @brief Makes the accumulation of 'max_i' quaternion rotation 'quat' and position 'pos' of a ar_track_alvar tag.
 * 
 * @param msg ar_track_alvar message.
 * @param i Current iteration.
 * @param max_i Maximim number of iterations.
 * @param [out] quat It contains the accumulation of 'i' quaternion rotations.
 * @param [out] pos It contains the accumulation of 'i' positions. 
 */
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

/**
 * @brief It publishes the biggest object and copies it to a global variable 'object' and its indices to 'objectInd' based on a segmentation of 'cloud'.
 * @details The segmentation extracts a cube, the inferior side plane set at 2 cm above the plane coefficient 'plane', the upper side in parallel to 'plane' but moved 'dist' meters, the  other sides based on the 'limits'.
 * 
 * @param limits It contains the corners of the tag.
 * @param pointcloud_pub point cloud publisher.
 */
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

/**
 * @brief Print available commands.
 */
void print_help() {
  cout << endl << "Welcome to the objects training node! You have the next options.\n\t[1] Update training set.\n\t[2] Update testing set.\n\t[3] Train model.\n\t[4] Try training.\n\t[5] Compute confusion matrix.\n\t[6] Change descriptor used.\n\t[7] Change Parameters.\n\t[8] Exit\n\nEnter an option: ";
}

/**
 * @brief Computes a confusion matrix
 * 
 * @param r Recogniser object already trained.
 * @param trainingHeader Objects of the training set.
 * @param testingHeader Objects of the testing set.
 * @param testingNames It contains the correct identifiers of the objects to predict.
 * @param testingObjects It contains the objects to predict.
 * @param testingIndices It contains the indices of the objects to predict.
 * @param [out] confMat Returned confusion matrix.
 */

void computeConfusionMatrix(const Recogniser &r, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &testingHeader, const std::vector<std::string> &testingNames, const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &testingObjects, const std::vector<pcl::PointIndices> &testingIndices, std::vector<std::vector<int>> &confMat) {
  
  int ntraining = trainingHeader.size();
  int ntesting = testingHeader.size();

  confMat = std::vector<std::vector<int>>(ntesting, std::vector<int>(ntraining, 0));

  for(int k = 0; k < testingNames.size(); k++) {
    std::string goodName = testingNames[k];
    std::string name = r.predict(testingObjects[k], testingIndices[k]);
  
    // Find position in confusion matrix.
    auto iti = std::find(testingHeader.begin(), testingHeader.end(), goodName);
    int i = std::distance(testingHeader.begin(), iti);

    auto itj = std::find(trainingHeader.begin(), trainingHeader.end(), name);
    int j = std::distance(trainingHeader.begin(), itj);

    confMat[i][j] += 1;
  }
}

/**
 * @brief It computes accuracy, precision, recalll and fmeasure from a confusion matrix.
 * 
 * @param confMat Confusion matrix.
 * @param trainingHeader Objects of the training set.
 * @param testingHeader Objects of the testing set.
 * @param [out] accur It returns the accuracy of each object.
 * @param [out] precision It returns the precision of each object.
 * @param [out] recall It returns the recall of each object.
 * @param [out] fmeasure It returns the f-measure of each object.
 */
void computeMetrics(const std::vector<std::vector<int>> &confMat, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &testingHeader, std::vector<float> &accur, std::vector<float> &precision, std::vector<float> &recall, std::vector<float> &fmeasure) {

  assert(testingHeader.size() == confMat.size() and testingHeader.size() > 0);
  assert(confMat[0].size() == trainingHeader.size());
  // Compute accuracy.
  int total = 0;
  for(int i = 0; i < confMat.size(); i++) {
    for(int j = 0; j < confMat[i].size(); j++) {
      total += confMat[i][j];
    }
  }

  // Compute precision.
  accur = std::vector<float>(trainingHeader.size(), 0.0);
  precision = std::vector<float>(trainingHeader.size(), 0.0);
  fmeasure = std::vector<float>(trainingHeader.size(), 0.0);
  recall = std::vector<float>(trainingHeader.size(), 0.0);
  for(int i = 0; i < testingHeader.size(); i++) {
    std::string goodName = testingHeader[i];

    auto itj = std::find(trainingHeader.begin(), trainingHeader.end(), goodName);
    int j = std::distance(trainingHeader.begin(), itj);

    int truePositive = confMat[i][j];

    int falsePositive = 0;
    for(int k = 0; k < testingHeader.size(); k++) {
      if(i != k) falsePositive += confMat[k][j];
    }

    int falseNegative = 0;
    for(int k = 0; k < trainingHeader.size(); k++) {
      if(j != k) falseNegative += confMat[i][k];
    }

    int trueNegative = total - (truePositive + falsePositive + falseNegative);

    accur[j] = ((float) truePositive + (float) trueNegative)/((float) total);
    precision[j] = (float) truePositive/((float) truePositive + (float) falsePositive);
    recall[j] = (float) truePositive/((float) truePositive + (float) falseNegative);
    fmeasure[j] = 2.0*(precision[j]*recall[j])/(precision[j] + recall[j]);
  }
}

/**
 * @brief It reads all the objects from a directory, where all the point clouds of an object are inside a subdirectory.
 * 
 * @param path Path of the root directory.
 * @param [out] objectsNames Vector containing the identifiers of the objects.
 * @param [out] objects Vector containing the points cloud of the objects.
 * @param [out] objectsIndices Vector containing the indexes of the objects.
 */
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

/**
 * @brief It saves an image every time the enter is pressed.
 * 
 * @param objects_path Path where the objects are stored.
 */
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

/**
 * @brief It predicts and object every time the enter is pressed.
 * 
 * @param ra Trained recognition class to make the predictions.
 */
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
  }
}

/**
 * @brief Makes the parsing of an option.
 * 
 * @param option Option identifier
 * @param objects_path Path of the objects directory.
 */
void parse_option(std::string option, std::string objects_path) {
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
        std::vector<std::string> trainingNames;
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> trainingObjects;
        std::vector<pcl::PointIndices> trainingIndices;
        readPointClouds(training_path, trainingNames, trainingObjects, trainingIndices);
        cout << "ended" << endl;
  
        // Train model and store it.
        r.addObjects(trainingObjects, trainingNames, trainingIndices);
        r.computeAll();
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
          std::vector<std::string> testingNames;
          std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> testingObjects;
          std::vector<pcl::PointIndices> testingIndices;

          std::cout << "Reading point clouds... " << std::flush;
          readPointClouds(testing_path, testingNames, testingObjects, testingIndices);
          std::cout << "ended" << std::endl;

          std::vector<std::string> trainingHeader = r.getTrainingNames();
          for(std::string s : trainingHeader) std::cout << s << " ";
          std::cout << endl;
          std::vector<std::string> testingHeader = computeHeader(testingNames);
          std::vector<std::vector<int>> confMat;

          std::cout << "Computing confusion matrix... " << std::flush;
          computeConfusionMatrix(r, trainingHeader, testingHeader, testingNames, testingObjects, testingIndices, confMat);
          std::cout << "ended" << std::endl;

          std::cout << "Computing metrics... " << std::flush;
          std::vector<float> precision;
          std::vector<float> recall;
          std::vector<float> fmeasure;
          std::vector<float> accur;
          computeMetrics(confMat, trainingHeader, testingHeader, accur, precision, recall, fmeasure);
          std::cout << "ended" << std::endl;

          cout << "Enter a file name: ";
          std::string name;
          getline(cin, name);
          writeMetrics(confMat, accur, precision, recall, fmeasure, trainingHeader, testingHeader, objects_path + "/" + name + ".csv");
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
      cout << "\t[7] Number H channel bins = " << r.getHBins() << endl;
      cout << "\t[8] Number S channel bins = " << r.getSBins() << endl;
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
        case '7':
          r.setHBins(stoi(value));
          break;
        case '8':
          r.setSBins(stoi(value));
          break;
      }
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "training");
  ros::NodeHandle nh;

  if(argc < 4){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./training <marker_size> <frame> <image_topic>" << endl;
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
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/training/markers", 50);
  ros::Publisher pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("training/object", 5);

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
  ROS_INFO("Published training area in /training/markers. Only the points inside it will be used for training");

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
  ROS_INFO("Published axis in /training/markers");

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

    parse_option(option, path);
    cout << "Enter an option: ";
  }
  spinner.stop();
}