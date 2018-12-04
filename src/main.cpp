#include <ros/ros.h>
#include <ros/console.h>
#include "Navigation.hpp"
#include "LaneDetect.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_mobile");
  ros::NodeHandle n;
	ros::Subscriber imageSub;
  Navigation mapMobile;
  LaneDetect follower;
  ros::Rate rate(50);
  imageSub = n.subscribe<sensor_msgs::Image>
    ("/camera/rgb/image_raw", 10, &LaneDetect::imageCallback, &follower);
  while(ros::ok()) {
    ros::spinOnce();
	//follower.detectLane();
    //mapMobile.move();
	rate.sleep();
  }


  return 0;
}
