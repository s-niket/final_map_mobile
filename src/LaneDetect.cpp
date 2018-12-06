#include <ros/ros.h>
#include <ros/console.h>
#include "LaneDetect.hpp"
#include <std_msgs/Float32.h>
#include <sstream>

using namespace cv;


LaneDetect::LaneDetect() {
  lanePub = nh.advertise < std_msgs::Float32 >("lane",1000); 
}

LaneDetect::~LaneDetect() {
  destroyAllWindows();
}

void LaneDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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

  cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
  ROS_INFO_STREAM("DID we get here");
  cvtColor(frame, frame_Gray, COLOR_BGR2GRAY);
	
  cv::Mat frame_threshold_white, frame_threshold_yellow, src, frame_mask;
  
  
  src = frame;
  
  // Detect the object based on HSV Range Values
  inRange(frame_HSV, Scalar(20, 16, 0), Scalar(30, 255, 255), frame_threshold_yellow);
  inRange(frame_Gray, 200, 255, frame_threshold_white);
  //Combine the two thresholds
  bitwise_or(frame_threshold_yellow,frame_threshold_white,frame_mask);
	//Gaussian Blur
  Mat gauss_gray;
  cv::Size kernel_size;
  kernel_size.height = 5;
  kernel_size.width = 5;
  GaussianBlur(frame_mask,gauss_gray, kernel_size,0,0,1);
  Mat edges;
  // Detect Edges
  Canny(gauss_gray,edges,0,50,3);
  //edges(Range(0,259),Range(0,363)).setTo(0);
  std::vector<Vec2f> lines;
  
	// Line Detection
  HoughLines(edges, lines, 1, CV_PI/180, 60, 0, 0 );
  float rho_right = 0;
  float rho_left = 0;
  float theta_right = 0;
  float theta_left = 0;
  float num_right = 0;
  float num_left = 0;
  // Line Averaging
  for( size_t i = 0; i < lines.size(); i++ )
  {
    float rho = lines[i][0];
	float theta = lines[i][1];
    if( theta <1.5708) {
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
  Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
  double a = cos(theta_right), b = sin(theta_right);
  double x0 = a*rho_right, y0 = b*rho_right;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  double mr = (1000*(a))/(1000*(-b));
  double br = (pt1.y-mr*pt1.x);
  line( src, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  
  a = cos(theta_left), b = sin(theta_left);
  x0 = a*rho_left, y0 = b*rho_left;
  pt3.x = cvRound(x0 + 1000*(-b));
  pt3.y = cvRound(y0 + 1000*(a));
  pt4.x = cvRound(x0 - 1000*(-b));
  pt4.y = cvRound(y0 - 1000*(a));
  double ml = (1000*(a))/(1000*(-b));
  double bl = (pt3.y-ml*pt3.x);
  line( src, pt3, pt4, Scalar(0,0,255), 3, CV_AA);
  // Lane Center
  pt5.y = -5;
  pt5.x = (((pt5.y-br)/mr)+((pt5.y-bl)/ml))/2;
  pt6.y = 800;
  pt6.x = (((pt6.y-br)/mr)+((pt6.y-bl)/ml))/2;
  double orient = 0;
  if (pt5.x != pt6.x) {
    double mc = (pt5.y - pt6.y)/(pt5.x - pt6.x);
    double bc = pt5.y-(mc*pt5.x);
    orient = (src.rows-bc)/mc;
  } else {
    orient = 0;
  }
  line( src, pt5, pt6, Scalar(0,255,0), 3, CV_AA);
  // Current Heading
  pt7.x = src.cols/2;
  pt7.y = -5;
  pt8.x = src.cols/2;
  pt7.y = 800;
  line( src, pt7, pt8, Scalar(255,0,0), 3, CV_AA);

  imshow("lines", src);
  //waitKey(0);
  std_msgs::Float32 laneData;
	ROS_INFO_STREAM("Orient: "<<orient);
  if (src.cols/2 < orient)
    laneData.data = -1;
  else if (src.cols/2 > orient)
    laneData.data = 1;
  else
    laneData.data = 0;
  lanePub.publish(laneData);
}

void LaneDetect::detectLane() {

  
	
}