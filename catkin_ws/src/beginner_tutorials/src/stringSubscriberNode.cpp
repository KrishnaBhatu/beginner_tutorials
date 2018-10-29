/**
 *  @copyright (c) BSD
 *  @file    stringSubscriberNode.cpp
 *  @author  Krishna Bhatu
 *
 *  @brief Subscriber Node
 *
 *  @section DESCRIPTION
 *
 *  Source code for subscriber node that subscribes to the /chatter topic
 *  and outputs that string message and prints it.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
/**
 * @brief A callback function whoch is called when a new message arrives at 
 *	  /chatter topic
 * @param msg It is a boost shared pointer that points to string message on 
 *	  /chatter topic
 * @return void
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Hey ROS: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
/**
  * Initializer function for the node which sees the command line arguments and   
  * performs any ROS arguments and name remapping, where the third argument is
  * the name of the node. 
  */
  ros::init(argc, argv, "listener");
/**
  * This fully initializes the node.
  */  
  ros::NodeHandle n;
/**
  * The subscriber() function defines the role of node to the ROS master.
  * The first parameter is the topic on which the node subscribes.
  * The second parameter is the size of buffer for input messages.
  * The third parameter is the function called when there is message on topic. 
  */  
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
/**
  * Enters an infinite loop which exits when master is shut down or when Ctrl+C  
  * is pressed.
  */  
  ros::spin();
  return 0;
}
