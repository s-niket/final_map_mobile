#include <ros/ros.h>
#include "SignDetect.hpp"
#include <sstream>
#include "std_msgs/String.h"

static const std::string OPENCV_WINDOW = "Image window";
std::string stopSignClassifier = "../classifiers/Stopsign_HAAR_19Stages.xml";
std::string speedLimitClassifier = "../classifiers/Speedlimit_HAAR_ 17Stages.xml";
std::string trafficLightClassifier = "../classifiers/TrafficLight_HAAR_16Stages.xml";
CascadeClassifier stopSign_cascade;
CascadeClassifier speedLimit_cascade;
CascadeClassifier trafficLight_cascade;
   
using namespace std;
using namespace cv;

SignDetect::SignDetect() {
  signPub = nh.advertise < std_msgs::String >("signs",1000); 
}

SignDetect::~SignDetect() {
  destroyAllWindows();
}
void SignDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
      // Convert from ROS Image msg to OpenCV image
      try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          frame = cv_ptr->image;
          waitKey(30);
      }
      catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              msg->encoding.c_str());
      }
}

void SignDetect(cv::Mat frame) {
  std::vector<Rect> stops;
  std::vector<Rect> speeds;
  std::vector<Rect> lights;
  Mat frame_gray;
  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );
  //-- Detect Stop Signs
  stopSign_cascade.detectMultiScale( frame_gray, stops, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
   for( size_t i = 0; i < stops.size(); i++ ) {
        Point center( stops[i].x + stops[i].width/2, stops[i].y + stops[i].height/2 );
        ellipse( frame, center, Size( stops[i].width/2, 
                                      stops[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );    
     }
    
  speedLimit_cascade.detectMultiScale( frame_gray, speeds, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
        for( size_t i = 0; i < speeds.size(); i++ ) {
            Point center( speeds[i].x + speeds[i].width/2, speeds[i].y + speeds[i].height/2 );
            ellipse( frame, center, Size( speeds[i].width/2, 
                                          speeds[i].height/2), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );    
         }
  trafficLight_cascade.detectMultiScale( frame_gray, lights, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
        for( size_t i = 0; i < lights.size(); i++ ) {
            Point center( lights[i].x + lights[i].width/2, lights[i].y + stops[i].height/2 );
            ellipse( frame, center, Size( lights[i].width/2, 
                                          lights[i].height/2), 0, 0, 360, Scalar( 255, 0, 0 ), 4, 8, 0 );    
         }
     

        //-- Show what you got
        imshow("Output", frame );
    }