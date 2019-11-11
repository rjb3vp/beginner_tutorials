// Bring in my package's API, which is what I'm testing
#include <ros/ros.h>
//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include "beginner_tutorials/SetRandomRange.h"

// Declare a test
TEST(TestSuite, testCase1)
{
//<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
  ros::NodeHandle n;
  ros::ServiceClient client =
n.serviceClient<beginner_tutorials::SetRandomRange>("random_data");

  beginner_tutorials::SetRandomRange srv;
  srv.request.mean = 3;
  srv.request.range = 2;
  bool call = client.call(srv);
  EXPECT_TRUE(call);

  if (call) {
    EXPECT_FALSE((bool)srv.response.error);
    //ROS_INFO_STREAM("Errors detected?:" << (bool)srv.response.error);
  }

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_talker");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
