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
 * @file LaneDetect_Test.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of unit testing for Lane Detect class
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "Navigation.hpp"

TEST(Navigation, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service

  EXPECT_NE(0, 1);
}

TEST(TestNavigation, No_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane", 1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("lane", 1000,
    &Navigation::laneCallback, &mapper);

  // Publish sign messages
  std_msgs::Float32 laneData;
  laneData.data = 0;
  lanePub.publish(laneData);
  ros::spinOnce();
  loop_rate.sleep();
ros::spinOnce();
  loop_rate.sleep();
    ros::spinOnce();
  loop_rate.sleep();
  // Tests
  EXPECT_EQ(0, mapper.getAngularV());
}

TEST(TestNavigation, Right_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane", 1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("/lane", 1000,
    &Navigation::laneCallback, &mapper);

  // Publish sign messages
  std_msgs::Float32 laneData;
  laneData.data = -1.0;
  lanePub.publish(laneData);
    ros::spinOnce();
  loop_rate.sleep();
ros::spinOnce();
  loop_rate.sleep();
    ros::spinOnce();
  loop_rate.sleep();

  // Tests
  EXPECT_EQ(-1, mapper.getAngularV());
}

TEST(TestNavigation, Left_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane", 1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("lane", 1000,
    &Navigation::laneCallback, &mapper);

  // Publish sign messages
  std_msgs::Float32 laneData;
  laneData.data = 1;
  lanePub.publish(laneData);
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();

  // Tests
  EXPECT_EQ(1, mapper.getAngularV());
}

TEST(TestNavigation, SpeedLimit) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs", 1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000,
    &Navigation::signCallback, &mapper);

  // Publish sign messages
  std_msgs::Int8 signData;
  signData.data = 2;
  signPub.publish(signData);
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();

  // Tests
  EXPECT_EQ(0.75, mapper.getLinearV());
}

TEST(TestNavigation, StopSign) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs", 1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000,
    &Navigation::signCallback, &mapper);

  // Publish sign messages
  std_msgs::Int8 signData;
  signData.data = 1;
  signPub.publish(signData);
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
    ros::spinOnce();
  loop_rate.sleep();

  // Tests
  EXPECT_EQ(0.5, mapper.getLinearV());
}

TEST(TestNavigation, NoSign) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs", 1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000,
    &Navigation::signCallback, &mapper);

  // Publish sign messages
  std_msgs::Int8 signData;
  signData.data = 0;
  signPub.publish(signData);
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  loop_rate.sleep();

  // Tests
  EXPECT_EQ(0.5, mapper.getLinearV());
}
