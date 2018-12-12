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
 * @file main.cpp
 * @author Niket Shah Zachary Zimits
 * @copyright 2018 BSD 3-clause
 * @brief Implementation of main function to navigate around unknown
 *        roads, detect lanes and follow them and detect signs
 */

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
  ros::Subscriber signSub;
  Navigation mapMobile;
  SignDetect detector;
  LaneDetect follower;
  ros::Rate rate(50);

  imageSub = n.subscribe<sensor_msgs::Image>
    ("/camera/rgb/image_raw", 500, &LaneDetect::imageCallback, &follower);

  imageStream = nh.subscribe < sensor_msgs::Image
      > ("/camera/rgb/image_raw", 10, &SignDetect::imageCallback, &detector);

  laneSub = n.subscribe("lane", 500, &Navigation::laneCallback, &mapMobile);

  signSub = n.subscribe("signs", 500, &Navigation::signCallback, &mapMobile);

  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
