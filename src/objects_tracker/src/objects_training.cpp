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

#include <mutex>
#include <thread>
#include <iostream>
#include <cstring>


#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <Eigen/Geometry>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGBA>());
bool newObject = false;
mutex m;

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

  pcl::PointIndices::ConstPtr minIndices = boost::make_shared<pcl::PointIndices>(*minClusterIndices);

  m.lock();
  pcl::copyPointCloud(*cloud, minIndices->indices, *object);
  newObject = true;
  m.unlock();

  pointcloud_pub.publish(*object);
}

void print_help() {
  cout << endl << "Welcome to the objects training node! You have the next options.\n\t[1] Update training set.\n\t[2] Update testing set.\n\t[3] Compute descriptors database.\n\t[4] Try training \n\t[5] Compute confusion matrix\n\nEnter an option: ";
}

void async_read(bool &newOption, std::string &option) {
  std::string current_option;
  while(true) {
    cin >> option;
    newOption = true;
  }
}

std::string findBestMatch(const pcl::PointCloud<pcl::VFHSignature308>::Ptr target, const std::vector<std::string> &descriptorNames, const pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr &kdtree) {

  std::map<std::string, int> objectMatches;
  for(int i = 0; i < target->points.size(); i++) {
    std::vector<int> nn_index(1);
    std::vector<float> nn_sqr_distance(1);
    kdtree->nearestKSearch(target->points[i], 1, nn_index, nn_sqr_distance);
    std::string name = descriptorNames[nn_index[0]];

    // Update matches
    if (objectMatches.find(name) == objectMatches.end()) objectMatches[name] = 1;
    else objectMatches[name] += 1;
  }

  // Search the object with most matches.
  std::string maxId = "";
  int maxValue = 0;
  int totalMatches = 0;
  for(auto& iter : objectMatches) {
    if(iter.second > maxValue) {
      maxId = iter.first;
      maxValue = iter.second;
    }
    totalMatches += iter.second;
  }
  assert(maxId != ""); 
  cout << "The object is '" << maxId << "', voting results : " << maxValue << "/" << totalMatches << endl;
  return maxId;
}

void computeConfusionMatrix(const pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr &kdtree, const pcl::PointCloud<pcl::VFHSignature308>::Ptr &testingDescriptors, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &trainingNames, const std::vector<std::string> &testingCount, std::vector<std::vector<int>> &confMat) {
  
  int nelem = trainingHeader.size();

  confMat = std::vector<std::vector<int>>(nelem, std::vector<int>(nelem, 0));

  int k = 0;
  for(int k1 = 0; k1 < testingCount.size(); k1+=2) {
    // It contains [object, numPhotos]

    std::string goodName = testingCount[k1];
    int c = stoi(testingCount[k1+1]);

    // Create target with its descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr target(new pcl::PointCloud<pcl::VFHSignature308>());
    target->points.insert(target->points.end(), &testingDescriptors->points[k],&testingDescriptors->points[k + c]);
    // Find best match.
    std::string name = findBestMatch(target, trainingNames, kdtree);

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

    k += c;
  }
}

void computeFiles(const std::string &path) {
  
  std::vector<std::string> objectsNames;
  pcl::PointCloud<pcl::VFHSignature308>::Ptr objectsDescriptors(new pcl::PointCloud<pcl::VFHSignature308>());
  std::vector<std::string> objectsCount;
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

      // Load pointcloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pcl::io::loadPCDFile<pcl::PointXYZRGBA>(rep.string(), *cloud);

      // Compute descriptors.
      pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>());
      computeDescriptors(cloud, descriptor);

      // Update descriptors and names.
      objectsDescriptors->points.insert(objectsDescriptors->points.end(), descriptor->points.begin(), descriptor->points.end());
      objectsNames.insert(objectsNames.end(), descriptor->points.size(), obsIt->path().stem().string());
      objectsCount.push_back(obsIt->path().stem().string() + " " + to_string(descriptor->points.size()));
    }

    cout << "Created descriptors for " <<  obsIt->path().filename() << endl;
  }

  assert(objectsNames.size() == objectsDescriptors->points.size());

  // Save descriptors.
  objectsDescriptors->height = 1;
  objectsDescriptors->width = objectsDescriptors->points.size();
  pcl::io::savePCDFileASCII (path + "/descriptors.pcd", *objectsDescriptors);

  // Save list of objects.
  writeList(objectsNames, path + "/objects_names");

  // Save list of photos for object.
  writeList(objectsCount, path + "/objects_count");

  cout << "Descriptors computed and stored!" << endl;
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

void tryTraining(const pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr &kdtree, const std::vector<std::string> &header) {
  cout << "Press enter every moment you want to take a picture, enter 'exit' to stop." << endl; 

  while(true) {
    std::string option;
    getline(cin, option);

    // Stop trying training.
    if(option == "exit") break;

    // Safe copy of the current point cloud.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objectCopy(new pcl::PointCloud<pcl::PointXYZRGBA>());
    m.lock();
    pcl::copyPointCloud(*object, *objectCopy);
    newObject = false;
    m.unlock();

    // Do nothing if the object is empty.
    if (objectCopy->points.size() == 0) {
      cout << "There is no object on the tag or it is to small." << endl;
      continue;
    }

    // Compute descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr target(new pcl::PointCloud<pcl::VFHSignature308>());
    computeDescriptors(object, target);

    findBestMatch(target, header, kdtree);
  }
}

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

void parse_option(std::string option, std::string objects_path, std::string frame) {
  if (option.length() != 1) return;

  switch(option[0]) {
    case '1':
      computeObjects(objects_path + "/training");
      break;

    case '2':
      computeObjects(objects_path + "/testing");
      break;

    case '3':
      cout << "Compute training descriptors" << endl;
      computeFiles(objects_path + "/training");

      cout << "Compute testing descriptors" << endl;
      computeFiles(objects_path + "/testing");
      break;

    case '4':
    case '5':
      {
        std::string training_path = objects_path + "/training";
        boost::filesystem::path dir(training_path);

        // Read objects names.
        std::vector<std::string> trainingNames;
        if (!readList(trainingNames, training_path + "/objects_names")) {
          cout << "ERROR loading objects_names" << endl;
          return;
        }
        cout << "Finished loading objects_names" << endl;

        // Read descriptors.
        pcl::PointCloud<pcl::VFHSignature308>::Ptr trainingDescriptors(new pcl::PointCloud<pcl::VFHSignature308>());
        if (pcl::io::loadPCDFile(training_path + "/descriptors.pcd" , *trainingDescriptors) == -1) {
          cout << "ERROR loading descriptors.pcd" << endl;
          return;
        }
        cout << "Finished loading descriptors.pcd" << endl;

        // Check if there are the same number of elements into the loaded structures
        if(trainingNames.size() != trainingDescriptors->points.size()) {
          cout << "ERROR: objects_names and descriptors.pcd have different number of elements" << endl;
          return;
        }

        // Check there is at least one element into the loaded structures.
        if(trainingNames.size() == 0 or trainingDescriptors->points.size() == 0) {
          cout << "ERROR: objects_names or descriptors.pcd are empty" << endl;
          return;
        }

        // Learn descriptors.
        pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::VFHSignature308>());
        kdtree->setInputCloud(trainingDescriptors);

        if(option[0] == '4') {
          tryTraining(kdtree, trainingNames);
        } else {
          std::string testing_path = objects_path + "/testing";

          // Read objects names.
          std::vector<std::string> testingCount;
          if (!readList(testingCount, testing_path + "/objects_count")) {
            cout << "ERROR loading objects_count" << endl;
            return;
          }
          cout << "Finished loading objects_count" << endl;

          std::vector<std::string> header = computeHeader(trainingNames);

          // Compute confusion matrix.
          std::vector<std::vector<int>> confMat;

          pcl::PointCloud<pcl::VFHSignature308>::Ptr testingDescriptors(new pcl::PointCloud<pcl::VFHSignature308>());
          if (pcl::io::loadPCDFile(testing_path + "/descriptors.pcd" , *testingDescriptors) == -1) {
            cout << "ERROR loading descriptors.pcd" << endl;
            return;
          }

          computeConfusionMatrix(kdtree, testingDescriptors, header, trainingNames, testingCount, confMat);

          cout << "Confusion matrix computed! Enter a file name: ";
          std::string name;
          getline(cin, name);
          writeConfusionMatrix(confMat, header, objects_path + "/" + name + ".csv");
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

  print_help();

  std::string option;
  
  std::string path = ros::package::getPath("objects_tracker");

  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();

  // Treatment of objects and options.
  while(true) {
    getline(cin, option);

    if (option == "-1") break;

    parse_option(option, path, frame);
    cout << "Enter an option: ";
  }
  spinner.stop();

  /*pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    estimateNormals(newObjectCloud, normals, 0.01);*/

    /*for(int i = 0; i < normals->points.size(); i++) {
      pcl::PointXYZRGBA p = newObjectCloud->points[i];
      pcl::Normal n = normals->points[i];

      std::vector< std::vector<double> > markerPos = {{p.x, p.y, p.z}, {p.x+n.normal_x, p.y+n.normal_y, p.z+n.normal_z}};
      marker_pub.publish(buildLineMarker(frame, 4 + i, markerPos, 0.001, color3));
    }*/

}