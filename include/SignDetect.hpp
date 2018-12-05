
#ifndef INCLUDE_SIGNDETECT_HPP_
#define INCLUDE_SIGNDETECT_HPP_


#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "sensor_msgs/Image.h"

using namespace cv;

class SignDetect {
 private:
  ros::NodeHandle nh;
  ros::Publisher signPub;
  ros::Subscriber imageLoop;
  cv_bridge::CvImagePtr cv_ptr;
  int flag;


 public:
  cv::Mat frame;

  SignDetect();

  ~SignDetect();

  void StartIP();

  void showFrame();

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void detectSign();

};


#endif  // INCLUDE_OBSTACLEDETECTOR_HPP_
