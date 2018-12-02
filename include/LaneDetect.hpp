
#ifndef INCLUDE_LANEDETECT_HPP_
#define INCLUDE_LANEDETECT_HPP_


#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"

using namespace cv;

class LaneDetect {
 private:
  ros::NodeHandle nh;
  ros::Publisher lanePub;
  
 public:
  cv::Mat frame;
  
  LaneDetect();
  
  ~LaneDetect();
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  void detectLane();
      
};

#endif 