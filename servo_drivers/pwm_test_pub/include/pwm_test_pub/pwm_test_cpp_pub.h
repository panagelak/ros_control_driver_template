/**
 * @file pwm_test_cpp_pub.h
 * @author Panagiotis Angelakis
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

#include <ros/ros.h>
// std msgs
#include <std_msgs/Int32MultiArray.h>
// Boost headers
#include <boost/shared_ptr.hpp>

/**
 * @brief This class contains the functionality to publish to Int32MultiArray type topic for the pwm driver
 */
class pwmTestCppPub {
public:
  /**
   * @brief Construct a new pwmTestCppPub object
   *
   * @param nh
   * @param topic_name
   */
  pwmTestCppPub(ros::NodeHandle &nh, const std::string &topic_name);
  /**
   * @brief Destroy the pwmTestCppPub object
   *
   */
  ~pwmTestCppPub();

private:
  /**
   * @brief A timer callback that gets called at a specific interval to publish the topic
   *
   */
  void publishTopicCallback(const ros::TimerEvent &e);

  ros::NodeHandle nh_;      /** node handle **/
  ros::Timer timerPublish_; /** timer object that calls publishTopicCallback at a specific rate **/
  std::string topic_name_;  /** topic name **/
  ros::Publisher int32_array_pub_;
  std_msgs::Int32MultiArray int32_array_msg_; /** int32 array msg **/
};
