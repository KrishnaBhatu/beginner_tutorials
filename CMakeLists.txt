cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## For compiling c++11 functions
add_compile_options(-std=c++11)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
## Find catkin macros and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

add_service_files(
  FILES
  changeString.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
## Declare catkin package 
catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(beginner_tutorialsPublisher src/stringPublisherNode.cpp)
target_link_libraries(beginner_tutorialsPublisher ${catkin_LIBRARIES})

add_executable(beginner_tutorialsSubscriber src/stringSubscriberNode.cpp)
target_link_libraries(beginner_tutorialsSubscriber ${catkin_LIBRARIES})

