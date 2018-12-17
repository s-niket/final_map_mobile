# 808XFinal

[![Build Status](https://travis-ci.com/zzimits/final_map_mobile.svg?branch=master)](https://travis-ci.com/zzimits/final_map_mobile)
[![Coverage Status](https://coveralls.io/repos/github/zzimits/808XFinal/badge.svg?branch=master)](https://coveralls.io/github/zzimits/808XFinal?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project will serve as the Final For ENPM808X software development for Robotics. One of the most important parts of an autonomous vehicle is having accurate maps that the robot can use as a reference. In an attempt to help provide this information this project is developing a autonomous vehicle that will travel along an unknown road and map the surrounding area. Inorder to do this safely the robot will have a lane keeping algorithm and sign detection algorithms that will allow it to behave saftly in its enviroment.

The final world is a two lane road that forms an oval with walls and obstacles on either side for the slam algorithm to pickup on. There are also several signs throughout the world that the robot interact with. There are several other worlds in the package the user can use to demonstrate more specific functionality.

<img src="https://raw.githubusercontent.com/zzimits/final_map_mobile/master/Images/world.png"> </img>


This software was developed in ROS Kinetic and Gazebo relying on the turtlebot 2 to simulate for our implementation. The code is witten in C++ and makes use of OpenCV 3.2 for several prebuilt computer vision algorithms. 
### Program Flow
The program follows the path shown in the diagram below.

<img src="https://raw.githubusercontent.com/zzimits/final_map_mobile/master/Images/UML.png"> </img>

The image is passed into the lane detector class and sign detector class through two seperate call backs. The lane detector class starts by coverting the image to HSV and selecting the yellow pixels through a threshold operation. After that a gausian filter is applied before canny edge detection is used reduce the image further. A Hough transform is used to find all of the line remaining in the image. The lines are divided into two groups based on their slope and then averaged to find the two borders of the current lane. These are averaged to find the center of the line and then proportional control is used to find an angular velocity command that will be sent to the navigation class.

<img src="https://raw.githubusercontent.com/zzimits/final_map_mobile/master/Images/lanes.png"> </img>

The sign detection algorithm uses Haar Cascade Classifiers to find signs in the input images. There are two different classifiers for stop signs and speed limit signs. The cascade function returns the position and region of interest for any signs in the image. Once the region of interest is large enough the sign detector will send a message to the navigator class informing it of the type of sign it has found. The method of training the cascades can be found here: https://docs.opencv.org/3.4/dc/d88/tutorial_traincascade.html

<img src="https://raw.githubusercontent.com/zzimits/final_map_mobile/master/Images/stop_yes_stop.png"> </img>

### Personnel
The development group is made up of Niket Shah and Zachary Zimits who also worked together on the 808XMidterm Project. Niket Shah got his undergad in Electronics and Telecomunication at Mumbai Univeristy. While Zachary Zimits majored in Mechanical Engineering at North Carolina State University.  

## Operation/Demo steps

This setup will assume that you already have a catkin workspace setup on your computer
```
cd <path to catkin_ws>/src
git clone https://github.com/zzimits/final_map_mobile
cd ..
catkin_make
source devel/setup.bash
```
	
Next the classifiers and test images must be copied into the ros enviroment so they can be accessed by the program.
```
cd ~/.ros
mkdir Images classifiers
cp <path to catkin ws>/Images/* Images
cp <path to catkin ws>/classifiers/* classifiers
```
To run the demo project, open two terminals, one for roscore(for map saving) and one for launching the demo:
```
Terminal 1: 
roscore

Terminal 2:
cd <path to catkin_ws>
roslaunch final_map_mobile demo.launch
```
The gazebo simulation starts in a paused state press play to begin the simulation.
On the Rviz, add Display for Map and subscribe to /map topic to view the generated map. Once satistfied, run the following commands by starting a new terminals: 
``` 
cd <path where you want to save the map> 
rosrun map_server map_saver -f <map-name>
```
The map files generated will consist of a .pgm and a .yaml file. The .pgm file can be viewed using the default image viewers.
Sample map is saved in the map folder of the project directory.
## Testing
To run tests
```
cd <path to catkin ws>
souce devel/setup.bash
catkin_make mapMobile_test
rostest final_map_mobile map_mobile_test.launch
```
To generate a coverage report
```
lcov --directory . --capture --output-file coverage.info
```
To view an HTML file of the output run
```
genhtml coverage.info --output-directory out
```

## Dependencies
Ubuntu 16.04
ROS Kinetic
Catkin
Gazebo
roscpp
rospy
std_msgs
geometry_msgs
sensor_msgs
move_base_msgs
cv_bridge
image_transport
## Known issues/bugs
If the gazebo world is started in the unpaused configuration the robot will start moving and drive off the road before the lane detection algorithm has time to initialize. By default the simulation starts paused requiring the user to start it. If the robot still drives off the road please wait a little longer before starting the simulation. Wait times vary by computer.

Occansionally the robot will experience a large jump in angular velocity, the cause is unknown, the lane tracking algorithm is usually capable of coping with the disturbance but if not reset the simulation. 

If the robot is experiencing large changes in z orientation when it approches a stop sign it may register and lose the sign multiple times causing the robot to stop several times at a single sign.
## API and other developer documentation

This project was done using Pair Programming. The development team took turns serving as driver and navigator with Niket Shah working on driver for the sign detection and Zachary Zimits working as the driver for the lane detection.

### Product Backlog Link<br/>
https://docs.google.com/spreadsheets/d/14xQGIgG6C0ESPju1HD3xA1dDB8FOpix7ZD7LX9gpMUo/edit?usp=sharing<br/>
####Sprint Planning Link<br/>
https://docs.google.com/spreadsheets/d/153FpOd2dDLhHrVDaHs887Bl2UnKq0t-d3hJzFEle1B4/edit?usp=sharing<br/>

### Doxygen Documentation
The generated doxygen documentation can be found at this link <br/>
https://htmlpreview.github.io/?https://raw.githubusercontent.com/zzimits/final_map_mobile/master/Results/html/index.html<br/>
Or in /Results/html/index.html file

## Video Presentation
Presentation: https://youtu.be/kymJSMamifE
Demonstration: https://youtu.be/kzqFy_jymck

## License

BSD 3 Clause 

Copyright 2018 Niket Shah Zachary Zimits

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.Copyright 2018 Zachary Zimits

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


