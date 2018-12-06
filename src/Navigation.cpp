#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "Navigation.hpp"

Navigation::Navigation() {
  velocity = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 500);

  msg.linear.x = 0.5;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  ros::Rate loop_rate(10);

}

Navigation::~Navigation() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

void Navigation::signCallback(){

}

void Navigation::laneCallback(const std_msgs::Float32::ConstPtr& lane) {

  float m = lane->data;
  if( m > 0) {
    msg.angular.z = m/2;
  } else if( m < 0) {
    msg.angular.z = m/2;
  } else
    msg.angular.z = 0;
  velocity.publish(msg);
}

void Navigation::move() {
  msg.linear.x = 0.5;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  velocity.publish(msg);


}
