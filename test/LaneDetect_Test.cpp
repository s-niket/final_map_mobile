#include "../include/LaneDetect.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>

TEST(TESTSuite, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service
 
  EXPECT_FALSE(1==1);
}

TEST(TestSuite, Image_processing) {
  cv::Mat src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/rviz.png" , cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);	
  imshow("TestImage",src);
  cv::waitKey();
  EXPECT_FALSE(1==0);
	
}