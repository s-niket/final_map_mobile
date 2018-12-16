# 808XFinal

[![Build Status](https://travis-ci.com/zzimits/final_map_mobile.svg?branch=master)](https://travis-ci.com/zzimits/final_map_mobile)
[![Coverage Status](https://coveralls.io/repos/github/zzimits/808XFinal/badge.svg?branch=master)](https://coveralls.io/github/zzimits/808XFinal?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project will serve as the Final For ENPM808X software development for Robotics. The purpose of this project is to design a frontier exploration robot that will explore an unknown space and react to certain stimuli in the enviromen. The exploration of the robot is going to be limited to the predefined road by means of a lane detection and tracking algorithm 

The proposed world is a two lane road that forms an oval with walls and obsticles on either side for the slam algorithm to pickup on. There are also several signs throughout the world that the robot interact with. The world was modeled in gazebo. 
### Program Flow


### Personel
The development group is made up of Niket Shah and Zachary Zimits who also worked together on the 808XMidterm Project. Niket Shah got his undergad in Electronics and Telecomunication at Mumbai Univeristy. While Zachary Zimits majored in Mechanical Engineering at North Carolina State University.  We are developing an autonomous robot that will navigate an enviroment marking the location of lanes and street signs. The software will be simulated in Gazebo on the turtlebot platform. Furthure details on the algorithms used and the preformance of the system will be added as the project developes.

## Operation/run/test/demo steps

This setup will assume that you already have a catkin workspace setup on your computer
```
git clone https://github.com/zzimits/final_map_mobile
cd <path to catkin ws>
source devel/setup.bash
catkin_make
```
	
Next the classifiers and test images must be copied into the ros enviroment so they can be accessed by the program.
```
cd ~/.ros
mkdir Images classifiers
cp <path to catkin ws>/Images/* Images
cp <path to catkin ws>/classifiers/* classifiers
```
###Testing
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

## Known issues/bugs
If the gazebo world is started in the unpaused configuration the robot will start moving and drive off the road before the lane detection algorithm has time to initialize. By default the simulation starts paused requiring the user to start it. If the robot still drives off the road please wait a little longer before starting the simulation. Wait times vary by computer.

Occansionally the robot will experience a large jump in angular velocity, the cause is unknown, the lane tracking algorithm is usually capable of coping with the disturbance but if not reset the simulation. 

If the robot is experiencing large changes in z orientation when it approches a stop sign it may register and lose the sign multiple times causing the robot to stop several times at a single sign.
## API and other developer documentation

Product Backlog Link<br/>
https://docs.google.com/spreadsheets/d/14xQGIgG6C0ESPju1HD3xA1dDB8FOpix7ZD7LX9gpMUo/edit?usp=sharing<br/>
Sprint Planning Link<br/>
https://docs.google.com/spreadsheets/d/153FpOd2dDLhHrVDaHs887Bl2UnKq0t-d3hJzFEle1B4/edit?usp=sharing<br/>

## Video Presentation

