cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## For compiling c++11 functions
add_compile_options(-std=c++11)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
## Find catkin macros and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation tf)

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
add_dependencies(beginner_tutorialsPublisher beginner_tutorials_generate_messages_cpp)

add_executable(beginner_tutorialsSubscriber src/stringSubscriberNode.cpp)
target_link_libraries(beginner_tutorialsSubscriber ${catkin_LIBRARIES})
add_dependencies(beginner_tutorialsSubscriber beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    unittest
    test/unittest.launch
    test/unittest.cpp
  )

  target_link_libraries(
    unittest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    unittest
    talker
    listener
    ${catkin_EXPORTED_TARGETS}
  )
endif()
