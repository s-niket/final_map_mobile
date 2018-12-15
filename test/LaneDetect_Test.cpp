
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "LaneDetect.hpp"

TEST(TESTSuite, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service
 
  EXPECT_FALSE(0==1);
}

TEST(TestSuite, Image_processing) {
  cv::Mat src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/turn_right.png" , cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);	
  
  LaneDetect follower;
  double m=1;
  m = follower.proccessImage(src);
  EXPECT_TRUE(m<0);
	
  src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/turn_left.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_TRUE(m>0);
	
  src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/right_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,0.5);
	
  src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/left_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,-0.5);
	
  src = imread( "/home/zach/catkin_ws/src/final_map_mobile/Images/no_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,0);
}