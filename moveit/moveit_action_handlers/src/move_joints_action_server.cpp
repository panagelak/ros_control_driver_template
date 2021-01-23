/**
 * @file move_joints_action_server.cpp
 * @author Panagiotis Angelakis
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "moveit_action_handlers/move_joints_action_server.h"

MoveJointsActionServer::MoveJointsActionServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : nh_(nh), nh_priv_(nh_priv) {}

MoveJointsActionServer::~MoveJointsActionServer(void) {}

bool MoveJointsActionServer::initialize() {

  // Get Parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "move_group", PLANNING_GROUP_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "action_server_name", action_name_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  const std::shared_ptr<tf2_ros::Buffer> dummy_tf;
  // get move_group
  ROS_INFO("Waiting for MoveIt servers to respond...");
  try {
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_, dummy_tf, ros::Duration(10));
  } catch (const std::runtime_error &e) {
    ROS_ERROR("[%s] %s Move Group has not Started Exiting...", action_name_.c_str(), e.what());
    return false;
  }
  // Initialize action server
  ROS_INFO_STREAM("Starting up the MoveToJointsMoveItActionServer ...  ");
  try {
    as_ptr_.reset(new MoveToJointsMoveItActionServer(
        nh_, action_name_, boost::bind(&MoveJointsActionServer::executeCallback, this, _1), false));
    as_ptr_->start();
  } catch (...) {
    ROS_ERROR_STREAM("MoveToJointsMoveItActionServer cannot not start.");
    return false;
  }

  move_group_->allowReplanning(true);
  move_group_->setStartStateToCurrentState();
  move_group_->setGoalTolerance(0.001);

  // get names of move group
  move_group_joint_names_ = move_group_->getJointNames();

  ROS_INFO_STREAM("Ready to receive goals!  ");

  return true;
}

void MoveJointsActionServer::executeCallback(const moveit_action_handlers::MoveToJointsMoveItGoalConstPtr &goal) {

  ros::WallTime _start;
  _start = ros::WallTime::now(); // Start timer
  ROS_INFO("[%s] MoveToJoints goal received", action_name_.c_str());

  // Check that the goal contains joint names corresponding to the move group
  for (moveit_action_handlers::PropertyValuePair joint_pair : goal->joint_pairs)
    if (!checkJointName(joint_pair.name)) {
      ROS_ERROR("[%s] Could not find joint: \"%s\". Unable to send goal.", action_name_.c_str(),
                joint_pair.name.c_str());
      feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::OPERATIONAL_EXCEPTION;
      as_ptr_->publishFeedback(feedback_);
      ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                  // publishing the result
      result_.success = false;
      result_.status = moveit_action_handlers::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_.millis_passed = feedback_.millis_passed;
      as_ptr_->setAborted(result_);
      return;
    }

  ROS_INFO("[%s] Planning frame: %s", action_name_.c_str(), move_group_->getPlanningFrame().c_str());
  ROS_INFO("[%s] End effector link: %s", action_name_.c_str(), move_group_->getEndEffectorLink().c_str());
  std::cout << "\n";

  move_group_->setMaxVelocityScalingFactor(double(goal->endEffectorVelocity));
  move_group_->setMaxAccelerationScalingFactor(double(goal->endEffectorAcceleration));
  move_group_->setPlanningTime(double(goal->timeoutSeconds));

  // get current state
  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  // get joint model group
  const robot_state::JointModelGroup *joint_model_group_ =
      move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

  // create a map of joints/names from joint_paies
  std::vector<double> joint_group_positions_;
  current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions_);
  std::map<std::string, double> name_value_pair = createNameValueMap(goal->joint_pairs);

  // set the joint target
  move_group_->setJointValueTarget(name_value_pair);

  // Planning thread
  plannig_thread_done_ = false;
  boost::thread planning_thread(&MoveJointsActionServer::moveit_planning, this);
  while (!plannig_thread_done_) {
    ros::Duration(0.001).sleep();
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::PLANNING;
    as_ptr_->publishFeedback(feedback_);

    if (as_ptr_->isPreemptRequested() || !ros::ok()) {
      move_group_->stop();
      feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::CANCELLING;
      as_ptr_->publishFeedback(feedback_);
      ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                  // publishing the result
      ROS_WARN("[%s] Preempted", action_name_.c_str());
      result_.success = false;
      result_.status = moveit_action_handlers::ActionResultStatusConstants::CANCELLED;
      result_.millis_passed = feedback_.millis_passed;
      as_ptr_->setPreempted(result_);
      return;
    }
  }

  // Planning has failed
  if (!planning_succeeded_) {
    ROS_ERROR("[%s] Planning joint goal FAILED", action_name_.c_str());
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    as_ptr_->publishFeedback(feedback_);
    ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                // publishing the result
    result_.success = false;
    result_.status = moveit_action_handlers::ActionResultStatusConstants::PLANNING_FAILED;
    result_.millis_passed = feedback_.millis_passed;
    as_ptr_->setAborted(result_);
    return;
  }
  // Planning Succeded
  ROS_INFO("[%s] Planning joint goal SUCCEEDED", action_name_.c_str());
  ROS_INFO("Visualizing plan to target: %s", planning_succeeded_ ? "SUCCEEDED" : "FAILED");

  // Execution thread
  execution_thread_done_ = false;
  boost::thread execution_thread(&MoveJointsActionServer::moveit_execution, this);
  while (!execution_thread_done_) {
    ros::Duration(0.001).sleep();

    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::EXECUTING;
    as_ptr_->publishFeedback(feedback_);

    if (as_ptr_->isPreemptRequested() || !ros::ok()) {
      move_group_->stop();
      feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::CANCELLING;
      as_ptr_->publishFeedback(feedback_);
      ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                  // publishing the result
      ROS_WARN("[%s] Preempted", action_name_.c_str());
      result_.success = false;
      result_.status = moveit_action_handlers::ActionResultStatusConstants::CANCELLED;
      result_.millis_passed = feedback_.millis_passed;
      as_ptr_->setPreempted(result_);
      return;
    }
  }

  // Execution has failed
  if (!execution_succeeded_) {
    move_group_->clearPathConstraints();
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    as_ptr_->publishFeedback(feedback_);
    ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                // publishing the result
    result_.success = false;
    result_.status = moveit_action_handlers::ActionResultStatusConstants::CONTROL_FAILED;
    result_.millis_passed = feedback_.millis_passed;
    as_ptr_->setAborted(result_);
    return;
  }
  // Execution Succeeded
  ROS_INFO("[%s] Succeeded", action_name_.c_str());
  feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
  as_ptr_->publishFeedback(feedback_);
  ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                              // publishing the result
  result_.success = true;
  result_.status = moveit_action_handlers::ActionResultStatusConstants::SUCCESS;
  result_.millis_passed = feedback_.millis_passed;
  as_ptr_->setSucceeded(result_);
}

bool MoveJointsActionServer::checkJointName(const std::string &name) {
  for (std::string available : move_group_joint_names_)
    if (name == available)
      return true;
  return false;
}

std::map<std::string, double> MoveJointsActionServer::createNameValueMap(
    const std::vector<moveit_action_handlers::PropertyValuePair> &joints_pairs) {
  std::map<std::string, double> map;
  for (auto joint_pair : joints_pairs)
    map.insert({joint_pair.name, joint_pair.value});
  return map;
}

void MoveJointsActionServer::moveit_planning() {
  planning_succeeded_ = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  plannig_thread_done_ = true;
}

void MoveJointsActionServer::moveit_execution() {
  execution_succeeded_ = (move_group_->execute(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  execution_thread_done_ = true;
}
