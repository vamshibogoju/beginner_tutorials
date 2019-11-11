/**
 * Copyright (c) 2019, Vamshi Kumar Bogoju
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *@file talkerTest.cpp
 *@author Vamshi Kumar Bogoju
 *@copyright BSD 3-Clause
 *@brief A ROS test which uses gtest framework to test the talker node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/baseOutputString.h"
#include "std_msgs/String.h"

/**
 * @brief      Tests whether the service exists
 * @param      testTalkerNode         gtest framework
 * @param      serviceExsistanceTest  Name of the test
 */
TEST(testTalkerNode, testServiceExsistance) {
  // Create node handle
  ros::NodeHandle node;

  // Register the client to the service
  ros::ServiceClient client = node.serviceClient
      < beginner_tutorials::baseOutputString > ("baseOutputString");

  // Service existance check
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief      Tests if changeText service can replace the text 
 * @param      testTalkerNode            gtest framework
 * @param      testServiceMessageUpdate  Name of the test
 */
TEST(testTalkerNode, testServiceMessageUpdate) {
  // Create node handle
  ros::NodeHandle node;

  // Register the client to the service
  ros::ServiceClient client = node.serviceClient
      < beginner_tutorials::baseOutputString > ("baseOutputString");
  // Initialize the service to srv object
  beginner_tutorials::baseOutputString srv;

  // change the input string
  srv.request.inputString = "testMessage";

  // request the server
  client.call(srv.request, srv.response);

  // tests to check whether the output and input are same
  EXPECT_STREQ("testMessage", srv.response.outputString.c_str());
}

