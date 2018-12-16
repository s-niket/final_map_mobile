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
 * @file Navigation.hpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Header of class Navigation to navigate around roads using lane
 *        and street signs information
 */

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

/*
 * @brief Class Navigation
 *        Implementation of navigation of turtlebot through
 *        unknown roads using the lane information and street signs
 *        information obtained by image processing
 */

class Navigation {
 private:
  // Node handler
  ros::NodeHandle nh;
  // Twist message for publishing velocities
  geometry_msgs::Twist msg;
  // Publisher for velocities
  ros::Publisher velocity;
  int inverse_flag = 0;

 public:
  /*
   * @brief Constructor for Navigation class
   *        Defines publisher for velocities and
   *        defines initial velocities
   * @param none
   * @return void
   */
  Navigation();

  /*
   * @brief Destructor for Navigation class
   *        Defines all velocities as zero as destruction of class
   * @param none
   * @return void
   */
  ~Navigation();

  /*
   * @brief Callback function for sending mesaages about sign detection
   * @param sign : int message about the type of message
   * @return int : number corresponding to the sign detected
   */

  void signCallback(const std_msgs::Int8::ConstPtr& sign);

  /*
   * @brief Callback function for sending messages about lane position
   * @param lane : float message for location of lane
   * @return void
   */
  void laneCallback(const std_msgs::Float32::ConstPtr& lane);

  /*
   * @brief Function to navigate turtlebot in +Z direction
   * @param none
   * @return void
   */
  void move();
};

#endif  // INCLUDE_NAVIGATION_HPP_
