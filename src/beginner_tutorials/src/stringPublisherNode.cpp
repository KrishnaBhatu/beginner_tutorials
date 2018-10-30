/**
  * BSD 3-Clause License

  * Copyright (c) 2018, KrishnaBhatu
  * All rights reserved.
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this
  * list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.

  * Neither the name of the copyright holder nor the names of its
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.

  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
