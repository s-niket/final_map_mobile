

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Navigation {
 private: 
  ros::NodeHandle nh;
  geometry_msgs::Twist msg;
  ros::Publisher velocity;
 
 public:
  Navigation();
  
  ~Navigation();
  
  void move();

};
