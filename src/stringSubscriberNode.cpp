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
 * @brief A callback function which is called when a new message arrives at 
 *	  /chatter topic
 * @param msg It is a boost shared pointer that points to string message on 
 *	  /chatter topic
 * @return void
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Hey ROS: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
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
