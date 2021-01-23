/**
 * @file pwm_test_cpp_pub.cpp
 * @author Panagiotis Angelakis
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <pwm_test_pub/pwm_test_cpp_pub.h>

// constructor
pwmTestCppPub::pwmTestCppPub(ros::NodeHandle &nh, const std::string &topic_name)
    : nh_(nh), topic_name_(std::move(topic_name)) {

  // instantiate publisher
  int32_array_pub_ = nh_.advertise<std_msgs::Int32MultiArray>(topic_name_, 10, true);

  // timer object that calls the timer callback at a specific rate  to publish the topic
  double hz = 10;
  timerPublish_ = nh_.createTimer(ros::Duration(1. / hz), &pwmTestCppPub::publishTopicCallback, this);

  // define message to publish

  // resize vector to size 16 and put -1
  int32_array_msg_.data.resize(16, -1);

  // no need!! to define layout for the pwm driver
  // int32_array_msg_.layout.data_offset = 4;
}

// destructor
pwmTestCppPub::~pwmTestCppPub() {
  // clear message
  int32_array_msg_.data.clear();
}

// this function gets called with a frequency of 10 hz
void pwmTestCppPub::publishTopicCallback(const ros::TimerEvent &e) {
  // change this values and then compile to test
  // if you dont want to publish to a servo delete the line (will publish -1)
  // this can go to the constructor too since we cannot change the values at runtime
  int32_array_msg_.data[0] = 5000;
  int32_array_msg_.data[1] = 5000;
  int32_array_msg_.data[2] = 5000;
  int32_array_msg_.data[3] = 5000;
  int32_array_msg_.data[4] = 5000;
  int32_array_msg_.data[5] = 5000;

  // gripper
  int32_array_msg_.data[6] = 5000;

  // publish the message
  int32_array_pub_.publish(int32_array_msg_);
}