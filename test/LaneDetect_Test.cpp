
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "LaneDetect.hpp"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


TEST(LaneDetect, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service
 
  EXPECT_FALSE(0==1);
}

TEST(LaneDetect, Image_processing) {
  cv::Mat src = imread( "Images/turn_right.png" , cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);	
  
  LaneDetect follower;
  double m=1;
  m = follower.proccessImage(src);
  EXPECT_TRUE(m<0);
	
  src = imread( "Images/turn_left.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_TRUE(m>0);
	
  src = imread( "Images/right_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,0.5);
	
  src = imread( "Images/left_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,-0.5);
	
  src = imread( "Images/no_lane.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,0);

  src = imread( "Images/big_left.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,-1);
	
  src = imread( "Images/big_right.png" , cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m,1);
}

TEST(LaneDetect, Image_callback) {
  LaneDetect follower;
  cv::Mat image = imread( "Images/turn_right.png" , cv::IMREAD_COLOR);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  follower.imageCallback(msg);
  EXPECT_NE(0,follower.getLaneData());
}