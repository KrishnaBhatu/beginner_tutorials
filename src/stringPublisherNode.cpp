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
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *  @copyright (c) BSD
  *  @file    stringPublisherNode.cpp
  *  @author  Krishna Bhatu
  *
  *  @brief Publisher Node with a service;
  *
  *  @section DESCRIPTION
  *
  *  Source code for publisher node that publishes to the /chatter topic
  *  and inputs that string message and it also has a service to change string.
  */
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"

/**
  * This is a structure for declaring global string.
  */ 
struct globstring {
  std::string str;
}message;
/**
 * @brief A change of string function which is called when a service is called 
 *        to change the publishing message
 * @param req It is the reference to the argument which the client
 *            requests the service which is the used as new publish message
 * @param resp It is the reference to the string that the service replies   
 * @return bool It returns true is the service is called successfully
 */

bool changeString(beginner_tutorials::changeString::Request &req,
                  beginner_tutorials::changeString::Response &resp) {
        resp.output = message.str + " Doing Unit Testing";
	/**
	  * Condition check for checking if the string passed to the service is 
	  * same as the current publishing string
	  */
        if (resp.output == req.a) {
                ROS_ERROR_STREAM("Change in String Not Achieved");
        } else {
                message.str = req.a;
                ROS_WARN_STREAM("The String is Modified");
        }
        return true;
}

int main(int argc, char **argv) {
/**
  * Initializer function for the node which sees the command line arguments and   
  * performs any ROS arguments and name remapping, where the third argument is
  * the name of the node. 
  */
  ros::init(argc, argv, "talker");
  message.str = "Hi This is Krishna";
/**
  * Check if the node is successfully initialized
  */  
  if (ros::isInitialized()) {
  int frequency = 100;
/**
  * Check for the command line argument for the frequency parameter
  */  
  if (argc == 2) {
    if (atoi(argv[1]) == 0) {
        ROS_WARN_STREAM("Frequency changed to non-zero value");
    } else {
      /**
        * frequency parameter is assigned with the argument value
        */
        frequency = atoi(argv[1]);
    }
  }
/**
  * This fully initializes the node.
  */  
  ros::NodeHandle n;
/**
  * Creating a service where the first argument of advertiseService() is the 
  * service name and the second argument is function name which defines the
  * service 
  */  
  ros::ServiceServer server  = n.advertiseService("changeString", changeString);
/**
  * The advertise() function defines the role of node to the ROS master.
  * The first parameter is the topic on which the node publishes.
  * The second parameter is the size of buffer for output messages.
  */  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
/**
  * Declaration of the implementation of the TransformBroadcaster provided by
  * the tf package.
  */  
  tf::TransformBroadcaster br;
/**
  * Declaration of the transform object and declare the information for
  * 3D transform.
  */  
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(10, 10, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, (3.14/2));
/**
  * Set the Rotation.
  */  
  transform.setRotation(q);
/**
  * Frequency at which it loops (10Hz)
  */  
  ros::Rate loop_rate(frequency);
/**
  * LOG message to monitor the frequency of looping
  */  
  ROS_DEBUG_STREAM("Frequency Check: %d"; frequency);
  while (ros::ok()) {
     /**
       * The transformation information is sent where we have,
       * First parameter: Transformation which was set above
       * Second parameter: Timestamp to the transformation
       * Third parameter: Name of the parent frame
       * Fourth parameter: Name of child frame
       */
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
                                           "talk"));
     /// Message object that has data to publish
     std_msgs::String msg;
     msg.data = message.str;
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
  } else {
     ROS_FATAL_STREAM("ROS is not initialized.");
  }
  return 0;
}
