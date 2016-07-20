// Ros includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Point cloud
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

/*! \file */

typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> MySyncPolicy;

bool newFrame = false;
bool first = true;
cv::Mat colorFrame, depthFrame, cameraMatrix;
cv::Mat lookupX, lookupY;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

/**
 * @brief It reads and stores the camera information.
 * 
 * @param cameraInfo Received camera information.
 * @param [out] cameraMatrix Place to store the information. 
 */
void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix)
{
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
}

/**
 * @brief It receives a synchronized colour and depth image and the camera info, and stores the point cloud. 
 * 
 * @param color Color image.
 * @param depth Depth image.
 * @param info Camera information.
 */
void callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& info) {
  colorFrame = cv_bridge::toCvCopy(color, "bgr8")->image;
  depthFrame = cv_bridge::toCvCopy(depth, "16UC1")->image;
  readCameraInfo(info, cameraMatrix);

  newFrame = true;
}

/**
 * @brief It creates a lookup to speed-up the point cloud generation.
 * 
 * @param width Width of the image.
 * @param height Height of the image.
 * @param cameraMatrix Camera matrix.
 * @param [out] lookupX X coordinates lookup.
 * @param [out] lookupY Y coordinates lookup.
 */
void createLookup(size_t width, size_t height, cv::Mat &cameraMatrix, cv::Mat &lookupX, cv::Mat &lookupY)
{
  const float fx = 1.0f / cameraMatrix.at<double>(0, 0);
  const float fy = 1.0f / cameraMatrix.at<double>(1, 1);
  const float cx = cameraMatrix.at<double>(0, 2);
  const float cy = cameraMatrix.at<double>(1, 2);
  float *it;

  lookupY = cv::Mat(1, height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}

/**
 * @brief It creates a point cloud from a depth and a colour image.
 * 
 * @param depth Depth image.
 * @param color Colour image.
 * @param [out] cloud Obtained point cloud.
 * @param lookupX X lookup.
 * @param lookupY Y lookup.
 */
void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const cv::Mat &lookupX, const cv::Mat &lookupY)
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
    const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      if(isnan(depthValue) || depthValue <= 0.001)
      {
        // not valid
        itP->x = itP->y = itP->z = badPoint;
        itP->rgba = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
      itP->a = 255;
    }
  }
}

/**
 * @brief It returns a ROS message containing a point cloud.
 * 
 * @param cloud Input cloud.
 * @param [out] cloud_msg Point cloud message.
 */
inline void getPointCloudMsg(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,  sensor_msgs::PointCloud2 &cloud_msg) {
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header = pcl_conversions::fromPCL((*cloud).header);
}

/**
 * @brief Node in charge of creating a point cloud from a colour and a depth image.
 * @details It reads the colour and depth images from the "/<cam>/<quality>/image_color_rect" and 
 * "/<cam>/<quality>/image_depth_rect" respectively, where <cam> and <quality> are node parameters. It also
 * needs the camera information obtained from the topic "/<cam>/<quality>+/camera_info". The Point cloud is
 * published to the topic "/<cam>/<quality>/PointCloud".
 * 
 * @param cam Camera identifier.
 * @param quality Image quality.
 * @param link Frame reference.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_image");
  ros::NodeHandle nh;

  if(argc < 4){
    std::cout << std::endl;
    cout << "Not enough arguments provided." << endl;
    cout << "Usage: ./point_cloud <cam> <quality> <link>" << endl;
    return 0;
  }

  std::string cam(argv[1]);
  std::string quality(argv[2]);
  std::string frame_id(argv[3]);

  std::string cam_color = "/" + cam + "/" + quality + "/image_color_rect";
  std::string cam_depth = "/" + cam + "/" + quality + "/image_depth_rect";
  std::string cam_info = "/" + cam + "/" + quality + "/camera_info";
  std::string cam_pointcloud = "/" + cam + "/" + quality + "/PointCloud";

  // Initialize camera subscribers.
  ROS_INFO("\nCamera subscribers:\n\t%s\n\t%s\n\t%s\n", cam_color.c_str(), cam_depth.c_str(), cam_info.c_str());
  message_filters::Subscriber<Image> color_sub(nh, cam_color, 1);
  message_filters::Subscriber<Image> depth_sub(nh, cam_depth, 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, cam_info, 1);

  // Initialize camera publishers.
  ROS_INFO("\nCamera PointCloud publisher: \n\t%s\n", cam_pointcloud.c_str());
  ros::Publisher cam_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > (cam_pointcloud, 1);

  // Set approximate policy and configure callback.
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_sub, depth_sub, info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

  Mat color, depth;

  while(ros::ok()) {
    if(newFrame) {

      color = colorFrame.clone();
      depth = depthFrame.clone();
      newFrame = false;

      if(first) {

        // Initialize cloud
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->height = color.rows;
        cloud->width = color.cols;
        cloud->is_dense = false;
        cloud->header.frame_id = frame_id;
        cloud->points.resize(cloud->height * cloud->width);

        createLookup(color.cols, color.rows, cameraMatrix, lookupX, lookupY);

        first = false;
      }

      sensor_msgs::PointCloud2 cloud_msg;
      
      // Publish point cloud.
      if (cam_pub.getNumSubscribers() > 0) {

        // Create the cloud.
        createCloud(depth, color, cloud, lookupX, lookupY);

        getPointCloudMsg(cloud, cloud_msg);
        cam_pub.publish(cloud_msg);
      }
    }
    ros::spinOnce();   
  }
}
