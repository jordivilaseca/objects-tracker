
// Ros includes
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// opencv inclues
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace cv_bridge;

void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // It does not make a copy, the image cannot be modified.
    cv::imshow("color_view_cam_1", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthCallback1(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat frame;
      frame = (cv_bridge::toCvCopy(msg, "16UC1")->image);
      double min, max;  
      minMaxIdx(frame, &min, &max);
      frame.convertTo(frame,CV_32F, 1.0/max);

      // Show frame.
      cv::imshow("depth_view_cam_1", frame);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
    }
  }

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // It does not make a copy, the image cannot be modified.
    cv::imshow("color_view_cam_2", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthCallback2(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat frame;
      frame = (cv_bridge::toCvCopy(msg, "16UC1")->image);
      double min, max;  
      minMaxIdx(frame, &min, &max);
      frame.convertTo(frame,CV_32F, 1.0/max);

      // Show frame.
      cv::imshow("depth_view_cam_2", frame);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
    }
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_image");
  ros::NodeHandle nh;
  cv::namedWindow("color_view_cam_1", WINDOW_NORMAL);
  cv::namedWindow("depth_view_cam_1", WINDOW_NORMAL);
  //cv::namedWindow("color_view_cam_2", WINDOW_NORMAL);
  //cv::namedWindow("depth_view_cam_2", WINDOW_NORMAL);
  cv::startWindowThread();
  
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber color_sub_cam_1 = it.subscribe("/cam1/qhd/image_color_rect", 1, imageCallback1);
  image_transport::Subscriber depth_sub_cam_1 = it.subscribe("/cam1/qhd/image_depth_rect", 1, depthCallback1);

  //image_transport::Subscriber color_sub_cam_2 = it.subscribe("/cam2/sd/image_color_rect", 1, imageCallback2);
  //image_transport::Subscriber depth_sub_cam_2 = it.subscribe("/cam2/sd/image_depth_rect", 1, depthCallback2);
  
  ros::spin();
  cv::destroyWindow("color_view_cam_1");
  cv::destroyWindow("depth_view_cam_1");
  //cv::destroyWindow("color_view_cam_2");
  //cv::destroyWindow("depth_view_cam_2");
}
