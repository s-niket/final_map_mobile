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
 * @brief Implementation of header SignDetect.hpp for detection
 *        of road signs
 */

#include <ros/ros.h>
#include <sstream>
#include "std_msgs/Int8.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "Navigation.hpp"
#include "SignDetect.hpp"

/*
 * @brief Constructor for class SignDetect
 *        Defines the publisher for the detected signs
 * @param none
 * @return void
 */

SignDetect::SignDetect() {
  signPub = nh.advertise < std_msgs::Int8 > ("signs", 100);
}

/*
 * @brief Destructor for class SignDetect
 *        Destroys all the open windows once the destructor is called
 * @param none
 * @return void
 */

SignDetect::~SignDetect() {
  cv::destroyAllWindows();
}

/*
 * @brief Callback function for camera topic subscriber and
 *        conversion of ROS images into OpenCV images and sign detection
 * @param msg : ROS image message
 * @return void
 */

int SignDetect::imageConvert(const sensor_msgs::ImageConstPtr& msg) {

      // Convert from ROS Image msg to OpenCV image
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          frame = cv_ptr->image;
      }
      catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              msg->encoding.c_str());
      }
  return detectSign(frame);
}

/*
 * @brief Function to detect signs in a particular frame
 *        The function uses OpenCV's Cascade Classifier to detect street signs
 * @param none
 * @return void
 */

int SignDetect::detectSign(cv::Mat frame) {
  // Defines the classifiers and checks if they are passed correctly
  if (!stopSign_cascade.load(stopSignClassifier)) {
    ROS_WARN_STREAM("--(!)Error loading Stop sign cascade\n");
  };
  if (!speedLimit_cascade.load(speedLimitClassifier)) {
    ROS_WARN_STREAM("--(!)Error loading Speed Limit cascade\n");
  };

  // To store positions of the detected signs
  std::vector < cv::Rect > stops;
  std::vector < cv::Rect > speeds;
  cv::Mat frame_gray;
  cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);
  std_msgs::Int8 signData;

  // Initialize signData to no sign
  signData.data = 0;

  // Stop sign detection

  stopSign_cascade.detectMultiScale(frame_gray, stops, 1.1, 2,
                                    0 | cv::CASCADE_SCALE_IMAGE,
                                    cv::Size(30, 30));
  for (size_t i = 0; i < stops.size(); i++) {
    cv::Point center(stops[i].x + stops[i].width / 2,
                     stops[i].y + stops[i].height / 2);
    if ((stops[i].width * stops[i].height) < 15000.0) {
      cv::ellipse(frame, center,
                  cv::Size(stops[i].width / 2, stops[i].height / 2), 0, 0, 360,
                  cv::Scalar(255, 0, 255), 4, 8, 0);
      cv::putText(frame, "Stop Sign", cv::Point(stops[i].x, stops[i].y),
                  cv::FONT_HERSHEY_PLAIN, 5, (0, 0, 0), 2, 8, false);
      if ((stops[i].width * stops[i].height) > 9500.0) {
        signData.data = 1;
        ROS_WARN_STREAM("Stop Sign Detected!");
      }
    }
  }

  // Speed limit sign detection

  speedLimit_cascade.detectMultiScale(frame_gray, speeds, 1.1, 2,
                                      0 | cv::CASCADE_SCALE_IMAGE,
                                      cv::Size(30, 30));
  for (size_t i = 0; i < speeds.size(); i++) {
    cv::Point center(speeds[i].x + speeds[i].width / 2,
                     speeds[i].y + speeds[i].height / 2);
    cv::ellipse(frame, center,
                cv::Size(speeds[i].width / 2, speeds[i].height / 2), 0, 0, 360,
                cv::Scalar(0, 0, 255), 4, 8, 0);
    cv::putText(frame, "Speed Limit", cv::Point(speeds[i].x, speeds[i].y),
                cv::FONT_HERSHEY_PLAIN, 5, (0, 0, 0), 2, 8, false);
    if ((stops[i].width * stops[i].height) < 20000.0) {
      signData.data = 2;
    }
    ROS_WARN_STREAM("Speed Limit Sign Detected!");
  }

  // Traffic light detection

  cv::imshow("Output", frame);
  signPub.publish(signData);
  return signData.data;
}
