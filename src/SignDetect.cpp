#include <ros/ros.h>
#include "SignDetect.hpp"
#include <sstream>
#include "std_msgs/String.h"
#include "opencv2/objdetect.hpp"
#include "Navigation.hpp"


using namespace std;
using namespace cv;
/*
void SignDetect::StartIP() {
  ros::Rate loop_rate(50);
  while (ros::ok) {
    //SignDetect::detectSign (frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
 */

SignDetect::SignDetect() {
  signPub = nh.advertise < std_msgs::String > ("signs", 100);
}

SignDetect::~SignDetect() {
  destroyAllWindows();
}
void SignDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

      // Convert from ROS Image msg to OpenCV image
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          frame = cv_ptr->image;
      }
      catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              msg->encoding.c_str());
      }
  std::string stopSignClassifier =
      "/home/niket/808X_Final_MapMobile/src/final_map_mobile/classifiers/Stopsign_HAAR_19Stages.xml";
  std::string speedLimitClassifier =
      "/home/niket/808X_Final_MapMobile/src/final_map_mobile/classifiers/Speedlimit_HAAR_ 17Stages.xml";
  std::string trafficLightClassifier =
      "/home/niket/808X_Final_MapMobile/src/final_map_mobile/classifiers/TrafficLight_HAAR_16Stages.xml";

  CascadeClassifier stopSign_cascade;
  CascadeClassifier speedLimit_cascade;
  CascadeClassifier trafficLight_cascade;

  if (!stopSign_cascade.load(stopSignClassifier)) {
    ROS_WARN_STREAM("--(!)Error loading Stop sign cascade\n");
  };
  if (!speedLimit_cascade.load(speedLimitClassifier)) {
    ROS_WARN_STREAM("--(!)Error loading Speed Limit cascade\n");
  };

  if (!trafficLight_cascade.load(trafficLightClassifier)) {
    ROS_WARN_STREAM("--(!)Error loading Traffic Light cascade\n");
  };

  std::vector < Rect > stops;
  std::vector < Rect > speeds;
  std::vector < Rect > lights;
  Mat frame_gray;
  cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
  //ROS_WARN_STREAM("Keep Going Son");
  equalizeHist(frame_gray, frame_gray);
  //-- Detect Stop Signs


  try {

  stopSign_cascade.detectMultiScale(frame_gray, stops, 1.1, 2,
                                    0 | CASCADE_SCALE_IMAGE, Size(30, 30));
  for (size_t i = 0; i < stops.size(); i++) {
    Point center(stops[i].x + stops[i].width / 2,
                 stops[i].y + stops[i].height / 2);
    ellipse(frame, center, Size(stops[i].width / 2, stops[i].height / 2), 0, 0,
            360, Scalar(255, 0, 255), 4, 8, 0);

  }

  speedLimit_cascade.detectMultiScale(frame_gray, speeds, 1.1, 2,
                                      0 | CASCADE_SCALE_IMAGE, Size(30, 30));
  for (size_t i = 0; i < speeds.size(); i++) {
    Point center(speeds[i].x + speeds[i].width / 2,
                 speeds[i].y + speeds[i].height / 2);
    ellipse(frame, center, Size(speeds[i].width / 2, speeds[i].height / 2), 0,
            0, 360, Scalar(0, 0, 255), 4, 8, 0);
  }

  trafficLight_cascade.detectMultiScale(frame_gray, lights, 1.1, 2,
                                        0 | CASCADE_SCALE_IMAGE, Size(30, 30));
  for (size_t i = 0; i < lights.size(); i++) {
    Point center(lights[i].x + lights[i].width / 2,
                   lights[i].y + lights[i].height / 2);
    ellipse(frame, center, Size(lights[i].width / 2, lights[i].height / 2), 0,
            0, 360, Scalar(255, 0, 0), 4, 8, 0);
  }
}
  catch (cv::Exception) {
    ROS_WARN_STREAM("Classifier Failed");
  }

  imshow("Output", frame);
  //waitKey(0);
}


void SignDetect::showFrame() {


}

void SignDetect::detectSign() {
  }
