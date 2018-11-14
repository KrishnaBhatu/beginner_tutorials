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
  *  @file    unittest.cpp
  *  @author  Krishna Bhatu
  *
  *  @brief Unit test for testing the "changeString" service 
  *
  *  @section DESCRIPTION
  *
  *  Source code for the unit testing of the "changeString service.
  */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/changeString.h"

/**
  * @brief  Test for checking the functionality of service
  *
  * @param  TestSuit: gtest framework where such suit can contain many tests.
  * @param  testofService: name of the test
  */
TEST(TestSuit, testofService) {
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
        ros::ServiceClient client = n.serviceClient
                                <beginner_tutorials::changeStri("changeString");
        /// set srv to be the object of service.
        beginner_tutorials::changeString srv;
        /// set the input text
        srv.request.a = "Doing Unit Testing";
        /// Call the service
        client.call(srv);
        /// check for response
        EXPECT_EQ("Hi This is Krishna Doing Unit Testing", srv.response.output);
}

int main(int argc, char** argv) {
    /**
      * Initializer function for the node which sees the command line arguments and   
      * performs any ROS arguments and name remapping, where the third argument is
      * the name of the node. 
      */
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
