/**
 * @file move_topose_handler_server.h
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
//#include <sys/ioctl.h> // For ioctl, TIOCGWINSZ
//#include <unistd.h>    // For STDOUT_FILENO
//#include <utility>

// action libs
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_action_handlers/ActionFeedbackStatusConstants.h>
#include <moveit_action_handlers/ActionResultStatusConstants.h>
#include <moveit_action_handlers/MoveToPoseMoveItAction.h>
#include <moveit_action_handlers/PoseStamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// moveit libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// For Visualization
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

// rosparam shortcuts
#include <rosparam_shortcuts/rosparam_shortcuts.h>

typedef actionlib::SimpleActionServer<moveit_action_handlers::MoveToPoseMoveItAction> MoveToPoseMoveItActionServer;

/**
 * @brief This handler accepts as goal a cartesian point relative to any tf
 * frame and will plan and execute a trajectory
 *
 */
class MoveToPoseActionServer {
public:
  /**
   * @brief Construct a new MoveToPoseActionServer object
   *
   * @param nh
   * @param nh_priv
   */
  MoveToPoseActionServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  /**
   * @brief Destroy the Move To Pose Handler Server object
   *
   */
  ~MoveToPoseActionServer(void);
  /**
   * @brief Initialization function
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize();

private:
  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see moveit_action_handlers::MoveToPoseMoveItGoal
   */
  void executeCallback(const moveit_action_handlers::MoveToPoseMoveItGoalConstPtr &goal);
  /**
   * @brief thread function for MoveIt planning
   *
   */
  void moveit_planning();
  /**
   * @brief thread function for MoveIt execution
   *
   */
  void moveit_execution();
  /**
   * @brief A function that transforms the pose goal relative to another frame
   *
   */
  geometry_msgs::PoseStamped changePoseFrame(const std::string &base_frame, const geometry_msgs::PoseStamped &goal_pose,
                                             bool &check_transform_ok);

  std::string name_ = "move_topose_action_server";
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::string action_name_;
  moveit_action_handlers::MoveToPoseMoveItFeedback feedback_;
  moveit_action_handlers::MoveToPoseMoveItResult result_;

  std::string PLANNING_GROUP_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  bool planning_succeeded_, execution_succeeded_, plannig_thread_done_, execution_thread_done_;

  ros::WallTime _start;
  boost::shared_ptr<MoveToPoseMoveItActionServer> as_ptr_;
  std::string base_link_name_, ee_link_name_;
};
