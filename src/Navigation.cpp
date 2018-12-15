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
 * @file Navigation.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of header Navigation.hpp to navigate roads
 *        using lane information and street signs information
 */

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include "Navigation.hpp"
#include <string>

/*
 * @brief Constructor for Navigation class
 *        Defines publisher for velocities and
 *        defines initial velocities
 * @param none
 * @return void
 */
Navigation::Navigation() {
  velocity = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 500);

  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  ros::Rate loop_rate(10);

}

/*
 * @brief Destructor for Navigation class
 *        Defines all velocities as zero as destruction of class
 * @param none
 * @return 
 */

Navigation::~Navigation() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

/*
 * @brief Destructor for Navigation class
 *        Defines all velocities as zero as destruction of class
 * @param none
 * @return void
 */

void Navigation::laneCallback(const std_msgs::Float32::ConstPtr& lane) {

  float m = lane->data;
  if( m > 0) {
    msg.angular.z = m;
	ROS_INFO_STREAM("Left: "<<msg.angular.z);
  } else if( m < 0) {
    msg.angular.z = m;
	ROS_INFO_STREAM("Right: "<<msg.angular.z);
  } else
      msg.angular.z = 0;
  velocity.publish(msg);
}

/*
 * @brief Callback function for sending mesaages about sign detection
 * @param sign : int message about the type of message
 * @return int : number corresponding to the sign detected
 */

void Navigation::signCallback(const std_msgs::Int8::ConstPtr& sign) {
  int flag = sign->data;
  ros::Rate loop_rate(50);
  
  if(flag != 0) {
	  if (flag == 1 && inverse_flag == 0) {
		ros::Time start = ros::Time::now();
		while (ros::Time::now() - start < ros::Duration(2.0)) {
		  msg.linear.x = 0.0;
		  msg.angular.z = 0.0;
		  velocity.publish(msg);
		  ROS_WARN_STREAM("STOPPING....");
		  velocity.publish(msg);
		  loop_rate.sleep();
		}
		msg.linear.x = 0.5;
		inverse_flag = 1;
	  } 
	  if (flag == 2) {
		ros::Time start = ros::Time::now();
		while (ros::Time::now() - start < ros::Duration(5.0)) {
		  ROS_WARN_STREAM("ALTERING SPEED");
		  msg.linear.x = 0.75;
		  velocity.publish(msg);
		}
	  }
  } else {
    inverse_flag = 0;
	ROS_WARN_STREAM("No Sign");
	msg.linear.x = 0.5;
  }

}

/*
  * @brief function for getting the angular velocity published to cmd_velocity
  * @param
  * @return double of angular velocity about the z axis
  */
double Navigation::getAngularV() {
  double value = msg.angular.z;
  return value;
}



double Navigation::getLinearV() {
  double value = msg.linear.x;
  return value;
}
	  
