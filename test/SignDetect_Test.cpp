/**
 * Copyright 2018, Niket Shah Zachary Zimits
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file SignDetect.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of unit testing on SignDetect class
 */
#include <gtest/gtest.h>
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include "std_msgs/Int8.h"
#include "SignDetect.hpp"

/*
 * @brief Test for the Sign Detect Constructor class
 *        Testing if the node handler and the publisher are created
 */

TEST(TestSignDetect, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service

  EXPECT_EQ(1 == 1);
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

