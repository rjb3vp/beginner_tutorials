/** @file talker_test.cpp
* @brief 2nd level tests for talker node
*
* Modified from the example
Copyright 2019 Ryan Bates, ROS.org

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "beginner_tutorials/SetRandomRange.h"

/**

* @brief Verifies our service

* @param TestSuite This is part of our test suite

* @param random_data_test This tests the random_data service

*/
TEST(TestSuite, random_data_test) {
  ros::NodeHandle n;
  ros::ServiceClient client =
  n.serviceClient<beginner_tutorials::SetRandomRange>("random_data");

  beginner_tutorials::SetRandomRange srv;
  srv.request.mean = 3;
  srv.request.range = 2;
  bool call = client.call(srv);
  // We expect the call to work, and give back no errors from the other side.
  EXPECT_TRUE(call);

  if (call) {
    EXPECT_FALSE((bool)srv.response.error);
  }
}


/**

* @brief Runs the test suite

* @param argc passthrough arg for ros_init

* @param argv passthrough arg for ros_init

*/
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
