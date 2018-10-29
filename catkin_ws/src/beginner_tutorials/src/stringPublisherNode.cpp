/**
 *  @copyright (c) BSD
 *  @file    stringPublisherNode.cpp
 *  @author  Krishna Bhatu
 *
 *  @brief Publisher Node;
 *
 *  @section DESCRIPTION
 *
 *  Source code for publisher node that publishes to the /chatter topic
 *  and inputs that string message.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
 
int main(int argc, char **argv)
{
/**
  * Initializer function for the node which sees the command line arguments and   
  * performs any ROS arguments and name remapping, where the third argument is
  * the name of the node. 
  */
  ros::init(argc, argv, "talker");
/**
  * This fully initializes the node.
  */  
  ros::NodeHandle n;
/**
  * The advertise() function defines the role of node to the ROS master.
  * The first parameter is the topic on which the node publishes.
  * The second parameter is the size of buffer for output messages.
  */  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
/**
  * Frequency at which it loops (10Hz)
  */  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
     /// Message object that has data to publish
     std_msgs::String msg;
     std::string str = "Hi This is Krishna";
     msg.data = str;
     /**
      *  The publish() function is used to send messages.
      */ 
     chatter_pub.publish(msg);
     ros::spinOnce();
     /**
      *  Sleep the publishing for the next iteration to maintain 10Hz publish
      *  rate.
      */
     loop_rate.sleep();
   }
  return 0;
 }
