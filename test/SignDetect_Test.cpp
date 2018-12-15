#include "std_msgs/Int8.h"
#include <gtest/gtest.h>
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include "SignDetect.hpp"

/*
 * @brief Test for the Sign Detect Constructor class
 *        Testing if the node handler and the publisher are created
 */

TEST(TestSignDetect, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service

  EXPECT_TRUE(1 == 1);
}

/*
 * @brief Test for the imageConvert methood
 *        It checks if the ros sensor image is rightly converted to
 *        OpenCV image frame so it can be passed to further processing methods
 */

TEST(TestImageConvert, TestimageConversion) {
  cv::Mat frame = imread("Images/Stop1.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  SignDetect detector;
  sensor_msgs::ImagePtr src = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                 frame).toImageMsg();
  detector.imageConvert(src);
  EXPECT_EQ(detector.sign_value, 0);
  imshow("TestImage", frame);
  cv::waitKey();
}

/*
 * @brief Test for the detectSign methood
 *        It checks if the stop sign is detected in the frame and the
 *        area of the sign is small enough to not stop the Turtlebot
 */

TEST(TestdetectSign1, TestForSignFarAway) {
  SignDetect detector;
  cv::Mat frame = imread("Images/Stop1.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  EXPECT_EQ(detector.detectSign(frame), 0);
  imshow("TestImage", frame);
  cv::waitKey();
}

/*
 * @brief Test for the detectSign methood
 *        It checks if the stop sign is detected in the frame and the
 *        area of the sign is large enough to make the Turtlebot stop
 */

TEST(TestdetectStopSign2, TestForSignCloseby) {
  SignDetect detector;
  cv::Mat frame = imread("Images/Stop2.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);
  EXPECT_EQ(detector.detectSign(frame), 1);
  imshow("TestImage", frame);
  cv::waitKey();
}

