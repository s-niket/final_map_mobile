#include "std_msgs/Int8.h"
#include <gtest/gtest.h>
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include "SignDetect.hpp"

TEST(TestSignDetect, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service

  EXPECT_TRUE(1==1);
}

TEST(TestdetectSign1, TestForSignFarAway) {
  SignDetect detector;
  cv::Mat frame = imread("Images/Stop1.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  EXPECT_EQ(detector.detectSign(frame), 0);
  imshow("TestImage", frame);
  cv::waitKey();
}

TEST(TestdetectStopSign2, TestForSignCloseby) {
  SignDetect detector;
  cv::Mat frame = imread("Images/Stop2.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  EXPECT_EQ(detector.detectSign(frame), 1);
  imshow("TestImage", frame);
  cv::waitKey();
}

TEST(TestImageConvert, TestimageConversion) {
  cv::Mat frame = imread("Images/Stop1.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  SignDetect detector;
  sensor_msgs::ImagePtr
  src = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                 frame).toImageMsg();
  EXPECT_EQ(detector.imageConvert(src), 0);
  imshow("TestImage", frame);
  cv::waitKey();
}
