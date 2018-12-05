#include <ros/ros.h>
#include <ros/console.h>
#include "Navigation.hpp"
#include "LaneDetect.hpp"
#include "SignDetect.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_mobile");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber imageSub;
  ros::Subscriber imageStream;
  ros::Subscriber laneSub;
  Navigation mapMobile;
  SignDetect detector;
  LaneDetect follower;
  ros::Rate rate(50);
  
  imageSub = n.subscribe<sensor_msgs::Image>
    ("/camera/rgb/image_raw", 10, &LaneDetect::imageCallback, &follower);
  
  imageStream = nh.subscribe < sensor_msgs::Image
      > ("/camera/rgb/image_raw", 10, &SignDetect::imageCallback, &detector);
  
  laneSub = n.subscribe("lane", 10, &Navigation::laneCallback, &mapMobile);
	
  while(ros::ok()) {
    ros::spinOnce();
	//follower.detectLane();
    //mapMobile.move();
	
	
	
	rate.sleep();
  }


  return 0;
}
