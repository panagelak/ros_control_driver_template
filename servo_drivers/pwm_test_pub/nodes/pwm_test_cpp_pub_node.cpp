/**
 * @file pwm_test_cpp_pub_node.cpp
 * @author Panagiotis Angelakis
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <csignal>
#include <ros/ros.h>

#include <pwm_test_pub/pwm_test_cpp_pub.h>

// unique smart pointer
std::unique_ptr<pwmTestCppPub> pwm_test_cpp_pub_ptr;

int main(int argc, char **argv) {

  const std::string node_name = "pwm_test_cpp_pub_node";
  const std::string topic_name = "/command";
  ros::init(argc, argv, node_name, ros::init_options::NoRosout);

  ros::NodeHandle nh("");

  ROS_INFO_STREAM("[" << node_name << "]"
                      << "Initializing node with ns :" << nh.getNamespace());

  // instansiate the smart pointer
  pwm_test_cpp_pub_ptr.reset(new pwmTestCppPub(nh, topic_name));

  // spin the node
  ros::spin();

  return 0;
}
