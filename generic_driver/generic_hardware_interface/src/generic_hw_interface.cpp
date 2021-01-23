#include <sstream>

#include <generic_hardware_interface/generic_hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace generic_hardware_interface {

GenericHardwareInterface::GenericHardwareInterface(ros::NodeHandle &nh)
    : nh_(nh), nh_priv_(nh, name_), position_controller_running_(true) {}

bool GenericHardwareInterface::init() {
  // Get Generic Hardware Interface parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "joints", joint_names_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "debug_mode", debug_mode_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "has_feedback", has_feedback_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Resize vectors
  num_joints_ = joint_names_.size();

  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  joint_position_command_.resize(num_joints_, 0.0);

  // Initialize Generic Robot driver
  // Object responsible for the communication with the robot
  robot_driver_ptr_.reset(new generic_robot_driver::GenericRobotDriver(nh_));
  bool robot_driver_initialized = robot_driver_ptr_->initialize();
  if (!robot_driver_initialized) {
    ROS_ERROR_STREAM("[generic_hw_interface] Failed to initialize robot driver");
    return false;
  }

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[generic_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {

      // Create joint state handle
      hardware_interface::JointStateHandle joint_state_handle = hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
      // Register handle into joint state interface
      joint_state_interface_.registerHandle(joint_state_handle);
      // Create joint position command handle
      hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
          joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]);
      // Register handle into joint position command interface
      position_joint_interface_.registerHandle(joint_handle);

    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[generic_hw_interface] " << e.what());
      return false;
    }
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  return true;
}

void GenericHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // assume perfect execution pass command into state
  if (!has_feedback_) {
    for (size_t i = 0; i < joint_position_command_.size(); i++) {
      joint_position_[i] = joint_position_command_[i];
    }
    return;
  }
  // put api in getJointPosition if you have feedback
  if (position_controller_running_) {
    robot_driver_ptr_->getJointPosition(joint_position_);
  }
}

void GenericHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  if (debug_mode_) {
    ;
  }
  if (position_controller_running_) {
    // maps and writes the command on the robot
    // here will just publish a custom topic with the mapped command
    robot_driver_ptr_->writeJointCommand(joint_position_command_);
  }
}

} // namespace generic_hardware_interface
