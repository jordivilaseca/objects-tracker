#include <mutex>

// Ros includes
#include "ros/ros.h"
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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, Image, Image, CameraInfo> MySyncPolicy;

bool newFrame = false;
bool first = true;
cv::Mat colorFrame1, depthFrame1, colorFrame2, depthFrame2, cameraMatrix1, cameraMatrix2;
cv::Mat lookupX1, lookupY1, lookupX2, lookupY2;
mutex m;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, cloud2, cloud;

void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix)
{
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
}

void callback(const sensor_msgs::ImageConstPtr& color1, const sensor_msgs::ImageConstPtr& depth1, const sensor_msgs::CameraInfoConstPtr& info1,
              const sensor_msgs::ImageConstPtr& color2, const sensor_msgs::ImageConstPtr& depth2, const sensor_msgs::CameraInfoConstPtr& info2) {
  m.lock();

  // Camera 1
  colorFrame1 = cv_bridge::toCvCopy(color1, "bgr8")->image;
  depthFrame1 = cv_bridge::toCvCopy(depth1, "16UC1")->image;
  readCameraInfo(info1, cameraMatrix1);

  // Camera 2
  colorFrame2 = cv_bridge::toCvCopy(color2, "bgr8")->image;
  depthFrame2 = cv_bridge::toCvCopy(depth2, "16UC1")->image;
  readCameraInfo(info2, cameraMatrix2);
  newFrame = true;

  m.unlock();
}

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

#define COLOR_IMAGE "image_color_rect"
#define DEPTH_IMAGE "image_depth_rect"
#define CAMERA_INFO "camera_info"

#define CAM1 "cam1"
#define CAM2 "cam2"
#define QUALITY_CAM "sd"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_image");
  ros::NodeHandle nh;

  // Initialize camera 1 subscribers.
  message_filters::Subscriber<Image> color1_sub(nh,"/" CAM1 "/" QUALITY_CAM "/" COLOR_IMAGE, 1);
  message_filters::Subscriber<Image> depth1_sub(nh,"/" CAM1 "/" QUALITY_CAM "/" DEPTH_IMAGE, 1);
  message_filters::Subscriber<CameraInfo> info1_sub(nh,"/" CAM1 "/" QUALITY_CAM "/" CAMERA_INFO, 1);

  // Initialize camera 2 subscribers.
  message_filters::Subscriber<Image> color2_sub(nh,"/" CAM2 "/" QUALITY_CAM "/" COLOR_IMAGE, 1);
  message_filters::Subscriber<Image> depth2_sub(nh,"/" CAM2 "/" QUALITY_CAM "/" DEPTH_IMAGE, 1);
  message_filters::Subscriber<CameraInfo> info2_sub(nh,"/" CAM2 "/" QUALITY_CAM "/" CAMERA_INFO, 1);

  // Initialize publisher.
  ros::Publisher pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > ("pointCloud", 1);

  // Set approximate policy and configure callback.
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color1_sub, depth1_sub, info1_sub, color2_sub, depth2_sub, info2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

  cameraMatrix1 = cv::Mat::zeros(3, 3, CV_64F);
  cameraMatrix2 = cv::Mat::zeros(3, 3, CV_64F);

  Mat color1, depth1, color2, depth2;

  while(ros::ok()) {
    if(newFrame) {

      m.lock();
      color1 = colorFrame1.clone();
      depth1 = depthFrame1.clone();
      color2 = colorFrame2.clone();
      depth2 = depthFrame2.clone();
      newFrame = false;
      m.unlock();

      if(first) {

        // Initialize cloud 1
        cloud1 = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud1->height = color1.rows;
        cloud1->width = color1.cols;
        cloud1->is_dense = false;
        cloud1->points.resize(cloud1->height * cloud1->width);

        //Initialize cloud 2
        cloud2 = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud2->height = color2.rows;
        cloud2->width = color2.cols;
        cloud2->is_dense = false;
        cloud2->points.resize(cloud2->height * cloud2->width);

        //Initialize cloud
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud->header.frame_id = "point_cloud";
        cloud->height = color1.rows;
        cloud->width = color1.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height * cloud->width);

        createLookup(color1.cols, color1.rows, cameraMatrix1, lookupX1, lookupY1);
        createLookup(color2.cols, color2.rows, cameraMatrix2, lookupX2, lookupY2);

        first = false;
      }

      createCloud(depth1, color1, cloud1, lookupX1, lookupY1);
      createCloud(depth2, color2, cloud2, lookupX2, lookupY2);
      //pcl::transformPointCloud(*cloud2, *cloud2, t); 
      //pcl::concatenateFields(*cloud1, *cloud2, *cloud);

      // Join point clouds.
      *cloud = *cloud1 + *cloud2;

      // Publish result.
      pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
      //sensor_msgs::PointCloud2 cloud_msg;
      //pcl::toROSMsg(*cloud, cloud_msg);
      pub.publish(cloud);
    }

    ros::spinOnce();   
  }
}
