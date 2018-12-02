#include <ros/ros.h>
#include "Navigation.hpp"
#include "LaneDetect.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_mobile");
  ros::NodeHandle n;
  ros::Subscriber imageSub;
  Navigation mapMobile;
  LaneDetect follower;
  
  imageSub = n.subscribe<sensor_msgs::Image>
    ("/camera/rgb/image_raw", 10, &LaneDetect::imageCallback, &follower);
  
	
	
  while(ros::ok()) {	
    follower.detectLane();
    mapMobile.move();
  }


  return 0;
}
