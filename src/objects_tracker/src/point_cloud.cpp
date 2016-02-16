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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

#define COLOR_IMAGE "image_color_rect"
#define DEPTH_IMAGE "image_depth_rect"
#define CAMERA_INFO "camera_info"

#define CAM1 "cam1"
#define CAM2 "cam2"
#define QUALITY_CAM "qhd"

#define CAM1_QUALITY "/" CAM1 "/" QUALITY_CAM "/"
#define CAM2_QUALITY "/" CAM2 "/" QUALITY_CAM "/"

#define CAM1_COLOR CAM1_QUALITY COLOR_IMAGE
#define CAM1_DEPTH CAM1_QUALITY DEPTH_IMAGE
#define CAM1_CAMERA CAM1_QUALITY CAMERA_INFO

#define CAM2_COLOR CAM2_QUALITY COLOR_IMAGE
#define CAM2_DEPTH CAM2_QUALITY DEPTH_IMAGE
#define CAM2_CAMERA CAM2_QUALITY CAMERA_INFO

#define CAM1_POINTCLOUD CAM1_QUALITY "PointCloud"
#define CAM1_POINTCLOUD_FILTER CAM1_POINTCLOUD "/filtered"

#define CAM2_POINTCLOUD CAM2_QUALITY "PointCloud"
#define CAM2_POINTCLOUD_FILTER CAM2_POINTCLOUD "/filtered"

#define ST_DEVIATION 1.0
#define KMEAN 20.0

typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, Image, Image, CameraInfo> MySyncPolicy;

bool newFrame = false;
bool first = true;
cv::Mat colorFrame1, depthFrame1, colorFrame2, depthFrame2, cameraMatrix1, cameraMatrix2;
cv::Mat lookupX1, lookupY1, lookupX2, lookupY2;
//mutex m;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, cloud2, cloud1Filt, cloud2Filt;

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
  // m.lock();

  // Camera 1
  colorFrame1 = cv_bridge::toCvCopy(color1, "bgr8")->image;
  depthFrame1 = cv_bridge::toCvCopy(depth1, "16UC1")->image;
  readCameraInfo(info1, cameraMatrix1);

  // Camera 2
  colorFrame2 = cv_bridge::toCvCopy(color2, "bgr8")->image;
  depthFrame2 = cv_bridge::toCvCopy(depth2, "16UC1")->image;
  readCameraInfo(info2, cameraMatrix2);
  newFrame = true;

  // m.unlock();
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

void filterCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, float mean, float stdeviation, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &filteredCloud) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor(true);
  sor.setInputCloud (cloud);
  sor.setMeanK (mean);
  sor.setStddevMulThresh (stdeviation);
  sor.filter(*filteredCloud);
}

inline void getPointCloudMsg(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,  sensor_msgs::PointCloud2 &cloud_msg) {
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header = pcl_conversions::fromPCL((*cloud).header);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_image");
  ros::NodeHandle nh;

  // Initialize camera 1 subscribers.
  ROS_INFO("\nCamera 1 subscribers:\n\t%s\n\t%s\n\t%s\n", CAM1_COLOR, CAM1_DEPTH, CAM1_CAMERA);
  message_filters::Subscriber<Image> color1_sub(nh, CAM1_COLOR, 1);
  message_filters::Subscriber<Image> depth1_sub(nh, CAM1_DEPTH, 1);
  message_filters::Subscriber<CameraInfo> info1_sub(nh, CAM1_CAMERA, 1);

  // Initialize camera 2 subscribers.
  ROS_INFO("\nCamera 2 subscribers:\n\t%s\n\t%s\n\t%s\n", CAM2_COLOR, CAM2_DEPTH, CAM2_CAMERA);
  message_filters::Subscriber<Image> color2_sub(nh, CAM2_COLOR, 1);
  message_filters::Subscriber<Image> depth2_sub(nh, CAM2_DEPTH, 1);
  message_filters::Subscriber<CameraInfo> info2_sub(nh, CAM2_CAMERA, 1);

  // Initialize publisher.
  ROS_INFO("\nCamera 1 PointCloud publisher: \n\t%s\n", CAM1_POINTCLOUD);
  ros::Publisher cam1_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > (CAM1_POINTCLOUD, 1);
  ROS_INFO("\nCamera 1 filtered PointCloud publisher: \n\t%s\n", CAM1_POINTCLOUD_FILTER);
  ros::Publisher cam1_filt_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > (CAM1_POINTCLOUD_FILTER, 1);
  ROS_INFO("\nCamera 2 PointCloud publisher: \n\t%s\n", CAM2_POINTCLOUD);
  ros::Publisher cam2_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > (CAM2_POINTCLOUD, 1);
  ROS_INFO("\nCamera 2 filtered PointCloud publisher: \n\t%s\n", CAM2_POINTCLOUD_FILTER);
  ros::Publisher cam2_filt_pub = nh.advertise< pcl::PointCloud<pcl::PointXYZRGBA> > (CAM2_POINTCLOUD_FILTER, 1);

  // Set approximate policy and configure callback.
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color1_sub, depth1_sub, info1_sub, color2_sub, depth2_sub, info2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

  cameraMatrix1 = cv::Mat::zeros(3, 3, CV_64F);
  cameraMatrix2 = cv::Mat::zeros(3, 3, CV_64F);

  Mat color1, depth1, color2, depth2;

  while(ros::ok()) {
    if(newFrame) {

      // m.lock();
      color1 = colorFrame1.clone();
      depth1 = depthFrame1.clone();
      color2 = colorFrame2.clone();
      depth2 = depthFrame2.clone();
      newFrame = false;
      // m.unlock();

      if(first) {

        // Initialize cloud 1
        cloud1 = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud1->height = color1.rows;
        cloud1->width = color1.cols;
        cloud1->is_dense = false;
        cloud1->header.frame_id = "cam1_link";
        cloud1->points.resize(cloud1->height * cloud1->width);

        //Initialize cloud 2
        cloud2 = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud2->height = color2.rows;
        cloud2->width = color2.cols;
        cloud2->is_dense = false;
        cloud2->header.frame_id = "cam2_link";
        cloud2->points.resize(cloud2->height * cloud2->width);

        // Initialize cloud 1 filtered
        cloud1Filt = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud1Filt->height = color1.rows;
        cloud1Filt->width = color1.cols;
        cloud1Filt->is_dense = false;
        cloud1Filt->header.frame_id = "cam1_link";
        cloud1Filt->points.resize(cloud1Filt->height * cloud1Filt->width);

        //Initialize cloud 2 filtered
        cloud2Filt = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud2Filt->height = color2.rows;
        cloud2Filt->width = color2.cols;
        cloud2Filt->is_dense = false;
        cloud2Filt->header.frame_id = "cam2_link";
        cloud2Filt->points.resize(cloud2Filt->height * cloud2Filt->width);

        createLookup(color1.cols, color1.rows, cameraMatrix1, lookupX1, lookupY1);
        createLookup(color2.cols, color2.rows, cameraMatrix2, lookupX2, lookupY2);

        first = false;
      }

      sensor_msgs::PointCloud2 cloud_msg;
      // Dealing with camera 1 publishers.
      if (cam1_pub.getNumSubscribers() > 0 or cam1_filt_pub.getNumSubscribers() > 0) {

        // Create the cloud.
        createCloud(depth1, color1, cloud1, lookupX1, lookupY1);

        // Publish pointCloud if necessary.
        if (cam1_pub.getNumSubscribers() > 0) {
          getPointCloudMsg(cloud1, cloud_msg);
          cam1_pub.publish(cloud_msg);
        }

        // Create the filtered pointCloud and publish it if necessary.
        if (cam1_filt_pub.getNumSubscribers() > 0) {
          filterCloud(cloud1, KMEAN, ST_DEVIATION, cloud1Filt);
          getPointCloudMsg(cloud1Filt, cloud_msg);
          cam1_filt_pub.publish(cloud_msg);
        }
      }

      // Dealing with camera 2 publishers.
      if (cam2_pub.getNumSubscribers() > 0 or cam2_filt_pub.getNumSubscribers() > 0) {

        // Create the cloud.
        createCloud(depth2, color2, cloud2, lookupX2, lookupY2);

        // Publish pointCloud if necessary.
        if (cam2_pub.getNumSubscribers() > 0) {
          getPointCloudMsg(cloud2, cloud_msg);
          cam2_pub.publish(cloud_msg);
        }

        // Create the filtered pointCloud and publish it if necessary.
        if (cam2_filt_pub.getNumSubscribers() > 0) {
          filterCloud(cloud2, KMEAN, ST_DEVIATION, cloud2Filt);
          getPointCloudMsg(cloud2Filt, cloud_msg);
          cam2_filt_pub.publish(cloud_msg);
        }
      }
    }

    ros::spinOnce();   
  }
}
