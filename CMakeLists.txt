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

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mapMobile src/main.cpp src/Navigation.cpp src/LaneDetect.cpp src/SignDetect.cpp)

target_link_libraries(mapMobile ${catkin_LIBRARIES} ${OpenCV_LIBS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(mapMobile_test test/map_mobile_test.launch test/main.cpp test/LaneDetect_Test.cpp test/SignDetect_Test.cpp src/LaneDetect.cpp src/SignDetect.cpp)
  target_link_libraries(mapMobile_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
  ##add_dependencies(mapMobile_test ${catkin_EXPORTED_TARGETS})


endif()
