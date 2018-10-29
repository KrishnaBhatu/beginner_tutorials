cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(beginner_tutorialsPublisher src/stringPublisherNode.cpp)
target_link_libraries(beginner_tutorialsPublisher ${catkin_LIBRARIES})

add_executable(beginner_tutorialsSubscriber src/stringSubscriberNode.cpp)
target_link_libraries(beginner_tutorialsSubscriber ${catkin_LIBRARIES})

