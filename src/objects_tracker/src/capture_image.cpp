#include <mutex>

// Ros includes
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// opencv inclues
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace message_filters;

bool newFrame1 = false;
bool newFrame2 = false;
cv::Mat colorFrame, depthFrame, dst;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // It does not make a copy, the image cannot be modified.
    colorFrame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    newFrame1 = true;
    cv::imshow("color_view_cam_1", colorFrame);
    cv::waitKey(2);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      depthFrame = (cv_bridge::toCvCopy(msg, "16UC1")->image);
      double min, max;  
      minMaxIdx(depthFrame, &min, &max);
      depthFrame.convertTo(depthFrame,CV_32F, 1.0/max);
      newFrame2 = true;

      // Show frame.
      cv::imshow("depth_view_cam_1", depthFrame);
      cv::waitKey(2);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
    }
  }

void callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& info) {
  
  // Process color frame.
  colorFrame = cv_bridge::toCvCopy(color, "bgr8")->image;
  cv::imshow("color_view_cam_1", colorFrame);
  cv::waitKey(2);

  // Process depth frame.
  depthFrame = (cv_bridge::toCvCopy(depth, "16UC1")->image);
  double min, max;  
  minMaxIdx(depthFrame, &min, &max);
  depthFrame.convertTo(depthFrame,CV_32F, 1.0/max);
  newFrame2 = true;

  // Show frame.
  cv::imshow("depth_view_cam_1", depthFrame);
  cv::waitKey(2);

  // Process overlapped frames
  for(uint i = 0; i < depthFrame.rows; i++) {
    for(uint j = 0; j < depthFrame.cols; j++) {
      if(depthFrame.at<float>(i, j) == 0) {
        dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
      } else {
        dst.at<Vec3b>(i,j) = colorFrame.at<Vec3b>(i,j);
      }
    }
  }
  cv::imshow("combination_view", dst);
  cv::waitKey(2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_image");
  ros::NodeHandle nh;
  cv::namedWindow("color_view_cam_1", WINDOW_NORMAL);
  cv::namedWindow("depth_view_cam_1", WINDOW_NORMAL);
  cv::namedWindow("combination_view", WINDOW_NORMAL);
  cv::startWindowThread();

  message_filters::Subscriber<Image> color1_sub(nh,"/cam1/qhd/image_color_rect", 1);
  message_filters::Subscriber<Image> depth1_sub(nh,"/cam1/qhd/image_depth_rect", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh,"/camera/rgb/camera_info", 1);

  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color1_sub, depth1_sub, info_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  
  /*image_transport::ImageTransport it(nh);
  image_transport::Subscriber color1_sub = it.subscribe("/cam1/qhd/image_color_rect", 1, imageCallback);
  image_transport::Subscriber depth1_sub = it.subscribe("/cam1/qhd/image_depth_rect", 1, depthCallback);
  message_filters::Subscriber info_cam_1 = nh.subscribe("/cam1/qhd/camera_info", 1, infoCallback);

  bool first = true;
  Mat dst;
  while(ros::ok()) {
    if (newFrame1 and newFrame2) {
      // Init dst frame.
      if(first) {
        ROS_INFO("Inside first");
        dst = Mat(depthFrame.rows, depthFrame.cols, CV_8UC3, Scalar(0,0,0));
        first = false;
      }
      
      for(uint i = 0; i < depthFrame.rows; i++) {
        for(uint j = 0; j < depthFrame.cols; j++) {
          if(depthFrame.at<float>(i, j) == 0) {
            dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
          } else {
            dst.at<Vec3b>(i,j) = colorFrame.at<Vec3b>(i,j);
          }
        }
      }
      newFrame1 = false;
      newFrame2 = false;

      cv::imshow("combination_view", dst);
      cv::waitKey(2);
    }
    ros::spinOnce();
  }*/
  cv::destroyWindow("color_view_cam_1");
  cv::destroyWindow("depth_view_cam_1");
  cv::destroyWindow("combination_view");
  //cv::destroyWindow("color_view_cam_2");
  //cv::destroyWindow("depth_view_cam_2");
}
