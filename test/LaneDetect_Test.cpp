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
 * @file LaneDetect_Test.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of unit testing for Lane Detect class
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "LaneDetect.hpp"

TEST(TESTSuite, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service

  EXPECT_NE(0 == 1);
}

TEST(TestSuite, Image_processing) {
  cv::Mat src = imread("Images/turn_right.png", cv::IMREAD_COLOR);
  cv::namedWindow("TestImage", cv::WINDOW_AUTOSIZE);

  LaneDetect follower;
  double m = 1;
  m = follower.proccessImage(src);
  EXPECT_LT(m < 0);

  src = imread("Images/turn_left.png", cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_GT(m > 0);

  src = imread("Images/right_lane.png", cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m, 0.5);

  src = imread("Images/left_lane.png", cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m, -0.5);

  src = imread("Images/no_lane.png", cv::IMREAD_COLOR);
  m = follower.proccessImage(src);
  EXPECT_EQ(m, 0);
}
