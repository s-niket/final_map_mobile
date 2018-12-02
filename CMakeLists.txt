cmake_minimum_required(VERSION 2.8.3)
project(final_map_mobile)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  move_base_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui ximgproc objdetect)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(mapMobile src/vision_node.cpp src/Navigation.cpp src/SignDetect.cpp)

target_link_libraries(mapMobile ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
