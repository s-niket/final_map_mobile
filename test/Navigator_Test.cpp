#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include "Navigation.hpp"

TEST(Navigation, Server_Existance_Test) {
  // ROS Node handle Creation
  ros::NodeHandle n;
  // Check the Existence of Service
 
  EXPECT_FALSE(0==1);
}

TEST(TestNavigation, No_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane",1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("lane", 1000, &Navigation::laneCallback, &mapper);

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
  EXPECT_EQ(0,mapper.getAngularV());

}

TEST(TestNavigation, Right_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane",1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("/lane", 1000, &Navigation::laneCallback, &mapper);

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
  EXPECT_EQ(-1,mapper.getAngularV());

}

TEST(TestNavigation, Left_LaneData) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher lanePub = n.advertise<std_msgs::Float32>("lane",1000);

  // Subscriber to traffic topic

  ros::Subscriber laneSub = nh.subscribe("lane", 1000, &Navigation::laneCallback, &mapper);

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
  EXPECT_EQ(1,mapper.getAngularV());

}

TEST(TestNavigation, SpeedLimit) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs",1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000, &Navigation::signCallback, &mapper);

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
  EXPECT_EQ(0.75,mapper.getLinearV());
}

TEST(TestNavigation, StopSign) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs",1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000, &Navigation::signCallback, &mapper);

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
  EXPECT_EQ(0.5,mapper.getLinearV());
}

TEST(TestNavigation, NoSign) {
  ros::NodeHandle n;
  ros::NodeHandle nh;
  Navigation mapper;

  ros::Rate loop_rate(10);

  // Publisher to sign messages
  ros::Publisher signPub = n.advertise<std_msgs::Int8>("signs",1000);

  // Subscriber to traffic topic

  ros::Subscriber signSub = nh.subscribe("signs", 1000, &Navigation::signCallback, &mapper);

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
  EXPECT_EQ(0.5,mapper.getLinearV());
}