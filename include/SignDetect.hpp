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
 * @file SignDetect.hpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Header for SignDetect class to detect road
 *        signs
 */

#ifndef INCLUDE_SIGNDETECT_HPP_
#define INCLUDE_SIGNDETECT_HPP_


#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int8.h"

/*
 * @brief Class SignDetect
 * Class defines attributes and methods needed for detection of
 * road signs using image processing using OpenCV libraries in C++
 */

class SignDetect {
 private:
  // Define node handler
  ros::NodeHandle nh;
  // Define sign publisher
  ros::Publisher signPub;
  // Define image subscriber
  ros::Subscriber imageLoop;
  // Define bridge for conversion of ROS image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  // Define flag for signs
  int flag;
  // OpenCV frame
  cv::Mat frame;
  // Stores the name of the classifiers stored in the directory /classifiers
  std::string stopSignClassifier = "classifiers/Stopsign_HAAR_19Stages.xml";
  std::string speedLimitClassifier = "classifiers/Speedlimit_HAAR_ 17Stages.xml";
  // To store the classifier data
  cv::CascadeClassifier stopSign_cascade;
  cv::CascadeClassifier speedLimit_cascade;





 public:
  int sign_value;
  /*
   * @brief Constructor for SignDetect class
   *        Defines the publisher and subscribers for the class
   * @param none
   * @return void
   */
  SignDetect();

  /*
   * @brief Destructor for SignDetect class
   *        Destroys all the OpenCV windows once the class is destroyed
   * @param none
   * @return void
   */

  ~SignDetect();

  /*
   * @brief Callback function for camera topic subscriber and
   *        conversion of ROS images into OpenCV images and sign detection
   * @param msg : ROS image message
   * @return void
   */

  int imageConvert(const sensor_msgs::ImageConstPtr& msg);

  /*
   * @brief Function to detect signs in a particular frame
   * @param none
   * @return void
   */

  int detectSign(cv::Mat frame);

};


#endif  // INCLUDE_OBSTACLEDETECTOR_HPP_
