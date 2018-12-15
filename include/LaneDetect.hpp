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
 * @file LaneDetection.hpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Header of class LaneDetect to detect lanes to navigate roads
 */

#ifndef INCLUDE_LANEDETECT_HPP_
#define INCLUDE_LANEDETECT_HPP_


#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <std_msgs/Float32.h>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"

/*
 * @brief Class LaneDetect
 *        Detect lanes using simple Hough Transform to separate lanes
 *        from roads using OpenCV
 */

class LaneDetect {
 private:
  // Node handler
  ros::NodeHandle nh;
  // Publisher for lane messages
  ros::Publisher lanePub;
  // Data for Publisher
  std_msgs::Float32 laneData;
 public:
  // CV frame for storing images
  cv::Mat frame;
  // CV frame after conversion to HSV
  cv::Mat frame_HSV;
  // CV frame after conversion to gray scale
  cv::Mat frame_Gray;

  /*
   * @brief Constructor for LaneDetect class
   *        Defines the publisher for lane detect messages
   * @param none
   * @return void
   */
  LaneDetect();

  /*
   * @brief Destructor for class LaneDetect
   *        Destroys all windows after class is destructed
   * @param none
   * @return void
   */
  ~LaneDetect();

  /*
   * @brief Callback function for image topic subscriber
   *        Converts the ROS images into OpenCV images and detects lanes
   * @param msg : ROS sensor image
   * @return void
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  double proccessImage(cv::Mat src);

   /*
   * @brief function for getting the error betweeen the current heading and current position
   * @param
   * @return double of the error
   */
  double getLaneData();
  
	
};

#endif
