/** @file talker.cpp
* @brief the ROS node that talks (publishes) messages
*
* Modified from the example
* Copyright 2019 ROS.org>
*/
#include <sstream>
#include <cstdlib>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "beginner_tutorials/SetRandomRange.h"
#include "beginner_tutorials/AddTwoInts.h"

int randomRange = 100;
int randomMean = 50;

bool add(beginner_tutorials::SetRandomRange::Request  &req,
         beginner_tutorials::SetRandomRange::Response &res)
{
  if (req.mean == 0) {
    ROS_WARN("Mean value of 0 is technically permitted, but probably a mistake.");
    ROS_INFO_STREAM("Mean updated to " << req.mean << " with range of " << req.range);
    res.error = false;
    randomRange = req.range;
    randomMean = req.mean;
  } 
  else if (req.mean < 0) {
    ROS_ERROR_STREAM("Mean cannot be " << req.mean << " which is < 0");
    res.error = true;
  }
  else {
    ROS_INFO_STREAM("Mean updated to " << req.mean << " with range of " << req.range);
    res.error = false;
    randomRange = req.range;
    randomMean = req.mean;
  }
  //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


bool callback(beginner_tutorials::SetRandomRange::Request& request, beginner_tutorials::SetRandomRange::Response& response)
{
  response.error = true;
  return true;
}

//bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
//{
//  return true;
//}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

//  ros::ServiceServer service = nh.advertiseService("my_service", callback);
  ros::ServiceServer service = n.advertiseService("random_dataOLD", callback);

  ros::ServiceServer service2 = n.advertiseService("random_data", add);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;


    if (randomRange < -1) {
      ROS_FATAL_STREAM("Impossible range of " << randomRange << " detected, failure imminent.");
    }

    int baseNumber = floor(randomRange / 2);

    ROS_DEBUG_STREAM("Calculated base number is " << baseNumber);

    int output = (rand() % randomRange) + (randomMean + floor(randomRange / 2));
    ss << "Our random value is=" << output;
    msg.data = ss.str();


    ROS_DEBUG("HELP");
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
