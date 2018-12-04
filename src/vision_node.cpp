#include <ros/ros.h>
#include "SignDetect.hpp"
#include "Navigation.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "detector");
  ros::NodeHandle n;
  ros::Subscriber imageStream;
  Navigation mapMobile;
  SignDetect detector;

  imageStream = n.subscribe < sensor_msgs::Image
      > ("/camera/rgb/image_raw", 10, &SignDetect::imageCallback, &detector);


  ros::Rate loop_rate(50);
  while (ros::ok) {
    //mapMobile.move();

    //signs.detectSign(frame);
    ros::spinOnce();
    loop_rate.sleep();

  }

  //signs.Signdetect(signs.frame);
  return 0;
}
