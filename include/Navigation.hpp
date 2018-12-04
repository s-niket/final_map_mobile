

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

class Navigation {
 private: 
  ros::NodeHandle nh;
  geometry_msgs::Twist msg;
  ros::Publisher velocity;
 
 public:
  Navigation();
  
  ~Navigation();
  
  void signCallback();

  void laneCallback(const std_msgs::String& lane);

  void move();

};
