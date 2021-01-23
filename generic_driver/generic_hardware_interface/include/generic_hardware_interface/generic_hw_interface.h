#pragma once

#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <generic_robot_driver/generic_robot_driver.h>

namespace generic_hardware_interface {

/**
 * @brief The GenericHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class GenericHardwareInterface : public hardware_interface::RobotHW {
public:
  /**
   * @brief Construct a new Generic Hardware Interface object
   */
  GenericHardwareInterface(ros::NodeHandle &nh);
  /**
   * @brief Destroy Generic Hardware Interface object
   */
  virtual ~GenericHardwareInterface() = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @param nh Root level ROS node handle
   * @param nh_local ROS node handle for the robot namespace
   * @returns True, if the setup was performed successfully
   *
   */
  virtual bool init();
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its PDL programs.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time &time, const ros::Duration &period) override;

protected:
  // Name of this class
  std::string name_ = "generic_hardware_interface";
  ros::NodeHandle nh_, nh_priv_;
  // Robot Driver Pointer
  boost::shared_ptr<generic_robot_driver::GenericRobotDriver> robot_driver_ptr_;
  // Configuration
  bool position_controller_running_;
  // Hardware Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  // States
  std::vector<double> joint_position_, joint_velocity_, joint_effort_;
  // Commands
  std::vector<double> joint_position_command_;
  // private
  size_t num_joints_;
  std::vector<std::string> joint_names_;
  // assume perfect execution if we have no feedback
  // pass command into state
  // this way you dont even need a real robot
  bool has_feedback_;

  bool debug_mode_;


};

} // namespace generic_hardware_interface