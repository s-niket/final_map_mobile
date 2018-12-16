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
 * @file LaneDetect.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of header LaneDetect.hpp for lane
 *        detection for navigation of roads
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include "LaneDetect.hpp"


/*
 * @brief Constructor for LaneDetect class
 *        Defines the publisher for lane detect messages
 * @param none
 * @return void
 */

LaneDetect::LaneDetect() {
  lanePub = nh.advertise < std_msgs::Float32 > ("lane", 1000);
}

/*
 * @brief Destructor for class LaneDetect
 *        Destroys all windows after class is destructed
 * @param none
 * @return void
 */

LaneDetect::~LaneDetect() {
  cv::destroyAllWindows();
}

/*
 * @brief Callback function for image topic subscriber
 *        Converts the ROS images into OpenCV images and detects lanes
 * @param msg : ROS sensor image
 * @return void
 */

void LaneDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
      // Convert from ROS Image msg to OpenCV image
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          frame = cv_ptr->image;
    cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              msg->encoding.c_str());
      }
  cv::Mat src;
  src = frame;
  proccessImage(src);
  imshow("lines", src);
}

double LaneDetect::proccessImage(cv::Mat src) {
  cv::Mat frame_HSV, frame_threshold_white, frame_threshold_yellow, frame_mask;
  // Detect the object based on HSV Range Values
  frame = src;
  cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
  inRange(frame_HSV, cv::Scalar(20, 49, 0), cv::Scalar(30, 255, 255),
          frame_threshold_yellow);
  // Combine the two thresholds
  // Bitwise_or(frame_threshold_yellow,frame_threshold_white,frame_mask);
  // Gaussian Blur
  cv::Mat gauss_gray;
  cv::Size kernel_size;
  kernel_size.height = 5;
  kernel_size.width = 5;
  GaussianBlur(frame_threshold_yellow, gauss_gray, kernel_size, 0, 0, 1);
  cv::Mat edges;
  // Detect Edges
  Canny(gauss_gray, edges, 0, 50, 3);
  std::vector < cv::Vec2f > lines;
  // Line Detection
  cv::HoughLines(edges, lines, 1, CV_PI / 180, 60, 0, 0);
  float rho_right = 0;
  float rho_left = 0;
  float theta_right = 0;
  float theta_left = 0;
  float num_right = 0;
  float num_left = 0;
  // Line Averaging
  for (size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0];
    float theta = lines[i][1];
    if (theta < 1.5708) {
      rho_left = rho_left + rho;
      theta_left = theta_left + theta;
      num_left++;
    } else {
      rho_right = rho_right + rho;
      theta_right = theta_right + theta;
      num_right++;
    }
  }
  rho_right = rho_right / num_right;
  theta_right = theta_right / num_right;
  rho_left = rho_left / num_left;
  theta_left = theta_left / num_left;
  // Line Graphing
  cv::Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  double a = cos(theta_right), b = sin(theta_right);
  double x0 = a*rho_right, y0 = b*rho_right;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  double mr = (1000*(a))/(1000*(-b));
  double br = (pt1.y-mr*pt1.x);
  line(src, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);

  a = cos(theta_left), b = sin(theta_left);
  x0 = a*rho_left, y0 = b*rho_left;
  pt3.x = cvRound(x0 + 1000*(-b));
  pt3.y = cvRound(y0 + 1000*(a));
  pt4.x = cvRound(x0 - 1000*(-b));
  pt4.y = cvRound(y0 - 1000*(a));
  double ml = (1000*(a))/(1000*(-b));
  double bl = (pt3.y-ml*pt3.x);
  line(src, pt3, pt4, cv::Scalar(0, 0, 255), 3, CV_AA);
  // Lane Center
  pt5.y = -5;
  pt5.x = (((pt5.y-br)/mr)+((pt5.y-bl)/ml))/2;
  pt6.y = 800;
  pt6.x = (((pt6.y-br)/mr)+((pt6.y-bl)/ml))/2;
  double orient = 0;
  double mc = 1;
  double bc = 0;
  ROS_INFO_STREAM("pt5: " << pt3.x << " pt6: " << pt2.x);
  if (pt5.x != pt6.x && pt5.y != pt6.y) {
    mc = (pt5.y - pt6.y)/(pt5.x - pt6.x);
    bc = pt5.y - (mc * pt5.x);
    orient = (src.rows - bc) / mc;
  } else {
    orient = 0;
  }
  ROS_INFO_STREAM("mc: " << mc << " bc: " << bc);
  line(src, pt5, pt6, cv::Scalar(0, 255, 0), 3, CV_AA);
  // Current Heading
  pt7.x = src.cols/2;
  pt7.y = -5;
  pt8.x = src.cols/2;
  pt7.y = 800;
  line(src, pt7, pt8, cv::Scalar(255, 0, 0), 3, CV_AA);

  std_msgs::Float32 laneData;
  if ((pt2.x < 10000 && pt2.x > -10000) && (pt3.x < -100000 || pt3.x > 100000))
    laneData.data = 0.5;
  else if ((pt3.x < 10000 && pt3.x > -10000)
      && (pt2.x > 10000 || pt2.x < -10000))
    laneData.data = -0.5;
  else if ((pt3.x > 10000 || pt3.x < -10000)
      && (pt2.x > 10000 || pt2.x < -10000))
    laneData.data = 0;
  else
    laneData.data = (-1 * (((src.rows - bc) / mc) - src.cols / 2) / 100);
  if (laneData.data > 1)
    laneData.data = 1;
  else if (laneData.data < -1)
    laneData.data = -1;
  lanePub.publish(laneData);
  ROS_WARN_STREAM("Data: " << laneData.data);
  return laneData.data;
}


