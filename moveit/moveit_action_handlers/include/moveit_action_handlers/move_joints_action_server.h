/**
 * @file move_joints_action_server.h
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
// action libs
#include <actionlib/server/simple_action_server.h>
#include <moveit_action_handlers/ActionFeedbackStatusConstants.h>
#include <moveit_action_handlers/ActionResultStatusConstants.h>
#include <moveit_action_handlers/MoveToJointsMoveItAction.h>

// moveit libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

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

typedef actionlib::SimpleActionServer<moveit_action_handlers::MoveToJointsMoveItAction> MoveToJointsMoveItActionServer;

/**
 * @brief This action accepts as goal a joint space point and will plan and
 * execute a trajectory
 *
 */
class MoveJointsActionServer {
public:
  /**
   * @brief Construct a new MoveJointsActionServer object
   *
   * @param nh
   * @param nh_priv
   */
  MoveJointsActionServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
  /**
   * @brief Destroy the Move Joints Handler Server object
   *
   */
  ~MoveJointsActionServer(void);
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
   * @param goal see moveit_action_handlers::MoveToJointsMoveItGoal
   */
  void executeCallback(const moveit_action_handlers::MoveToJointsMoveItGoalConstPtr &goal);
  /**
   * @brief Checks if the joint name on the goal is appropiate
   *
   * @return true
   * @return false if can not find the joint name
   */
  bool checkJointName(const std::string &name);
  /**
   * @brief Creates a dictionary joint_name with joint value
   *
   */
  std::map<std::string, double> createNameValueMap(const std::vector<moveit_action_handlers::PropertyValuePair> &goal);
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


  // Name of this class
  std::string name_ = "move_joint_action_server";
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  std::string action_name_;
  moveit_action_handlers::MoveToJointsMoveItFeedback feedback_;
  moveit_action_handlers::MoveToJointsMoveItResult result_;

  std::string PLANNING_GROUP_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  bool planning_succeeded_, execution_succeeded_, plannig_thread_done_, execution_thread_done_;

  std::vector<std::string> move_group_joint_names_;

  boost::shared_ptr<MoveToJointsMoveItActionServer> as_ptr_;
};
