#pragma once

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <ros/time.h>

// used to send to the pwm9685 driver
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
// activate arm service
#include <arm_msgs/ActivateArm.h>

namespace generic_robot_driver {
/**
 * @brief This is the main class for interfacing with your Robot's API
 */
class GenericRobotDriver {
public:
  /**
   * @brief Construct a new Generic Driver object
   *
   * @param nh global ROS NodeHandle
   * @param nh_local local ROS NodeHandle
   */
  GenericRobotDriver(ros::NodeHandle &nh);
  /**
   * @brief Destroy the Generic Driver object
   */
  ~GenericRobotDriver();
  /**
   * @brief
   * @return true if generic driver initialized correctly
   */
  bool initialize();
  /**
   * @brief Write Joint Position Command to the Robot
   */
  void writeJointCommand(const std::vector<double> &joint_command);
  /**
   * @brief Read the robot Joint Position
   */
  void getJointPosition(std::vector<double> &joint_position);

  // ROBOT SPECIFIC API
  /**
   * @brief Routine Service function to activate the arm
   */
  bool activate_arm_routine(arm_msgs::ActivateArm::Request &req, arm_msgs::ActivateArm::Response &res);
  /**
   * @brief Transforms the Ros control command into robot specific command
   */
  void transform_command(const std::vector<double> &joint_command, std_msgs::Int32MultiArray &servo_command);
  /**
   * @brief Map a range of values into another range of values
   */
  float map(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange,
            bool reverse_direction);

private:
  std::string name_ = "generic_robot_driver"; // for parameter reading
  ros::NodeHandle nh_, nh_priv_;              /**< ROS NodeHandle objects required for parameters reading */

  // ROBOT SPECIFIC API

  // vector for the 16 int values we need to publish to the robot
  std_msgs::Int32MultiArray servo_command_;
  // Subscriber to activate the write capability
  // e.g you might want to move the arm in pseudo mode to a specific location before activating the robot
  ros::ServiceServer activate_arm_service_;
  // real time publishes ( publishes in a different thread as not to block the ros control thread)
  // this publisher publish an array of size 16 that the pwm9685 driver expects to move the servos
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Int32MultiArray>> servo_command_pub_;


  // configuration values

  // location on pwm driver board (16 slots)
  std::vector<int> servo_pins_;

  // gripper has two joints on the urdf which can be moved indepedently
  // you can put only one joint if you use mimic joints for the fingers
  // but one servo we listen only to one joint to transform

  // ros lower joint limits
  std::vector<float> joint_lower_;
  // ros upper joint limits
  std::vector<float> joint_upper_;
  // servo command lower limits
  std::vector<float> servo_lower_;
  // servo command upper limits
  std::vector<float> servo_upper_;
  // servo command offsets
  std::vector<float> offsets_;
  // reverse direction
  std::vector<bool> reverse_direction_;

  // servos 1 and 2 depedented
  bool has_close_loop_;
  // is arm activated?
  bool arm_activated_;
};

} // namespace generic_robot_driver
