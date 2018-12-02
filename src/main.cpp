#include <ros/ros.h>
#include "Navigation.hpp"
#include "SignDetect.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "map_mobile");

  Navigation mapMobile;
  mapMobile.move();

  SignDetect signs();

  return 0;
}
