/**
 * @file move_topose_handler_server.cpp
 * @author Panagiotis Angelakis
 * @brief
 * @version 0.1
 * @date 03-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "moveit_action_handlers/move_topose_action_server.h"

MoveToPoseActionServer::MoveToPoseActionServer(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
    : nh_(nh), nh_priv_(nh_priv) {}

MoveToPoseActionServer::~MoveToPoseActionServer(void) {}

bool MoveToPoseActionServer::initialize() {
  // Get Parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "move_group", PLANNING_GROUP_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "action_server_name", action_name_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "base_link_name", base_link_name_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "ee_link_name", ee_link_name_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  const std::shared_ptr<tf2_ros::Buffer> dummy_tf;

  // get move_group
  ROS_INFO("Waiting for MoveIt servers to respond...");
  try {
    move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_, dummy_tf, ros::Duration(10));
  } catch (const std::runtime_error &e) {
    ROS_ERROR("[%s] %s Move Group has not started yet exiting.", action_name_.c_str(), e.what());
    return false;
  }

  // Initialize action server
  ROS_INFO_STREAM("Starting up the MoveToPoseMoveItActionServer ...  ");
  try {
    as_ptr_.reset(new MoveToPoseMoveItActionServer(
        nh_, action_name_, boost::bind(&MoveToPoseActionServer::executeCallback, this, _1), false));
    as_ptr_->start();
  } catch (...) {
    ROS_ERROR_STREAM("MoveToPoseMoveItActionServer cannot not start.");
    return false;
  }

  // moveit default parameters
  move_group_->allowReplanning(true);
  move_group_->setStartStateToCurrentState();
  move_group_->setGoalTolerance(0.001);

  ROS_INFO_STREAM("Ready to receive goals!  ");
  return true;
}

void MoveToPoseActionServer::executeCallback(const moveit_action_handlers::MoveToPoseMoveItGoalConstPtr &goal) {
  ros::WallTime _start;
  _start = ros::WallTime::now(); // Start timer

  ROS_INFO("[%s] MoveToPoSE goal received", action_name_.c_str());
  ROS_INFO("[%s] Planning frame: %s", action_name_.c_str(), move_group_->getPlanningFrame().c_str());
  ROS_INFO("[%s] End effector link: %s", action_name_.c_str(), move_group_->getEndEffectorLink().c_str());
  printf("\n");

  move_group_->setMaxVelocityScalingFactor(double(goal->endEffectorVelocity));
  move_group_->setMaxAccelerationScalingFactor(double(goal->endEffectorAcceleration));
  move_group_->setPlanningTime(double(goal->timeoutSeconds));
  move_group_->clearPoseTargets();

  // euler pose to quaternion pose
  geometry_msgs::PoseStamped target_pose;
  tf2::Quaternion quat;
  target_pose.header = goal->target_pose.header;
  target_pose.pose.position = goal->target_pose.pose.position;
  quat.setRPY(goal->target_pose.pose.orientation.roll, goal->target_pose.pose.orientation.pitch,
              goal->target_pose.pose.orientation.yaw);
  quat.normalize();
  target_pose.pose.orientation.x = quat[0];
  target_pose.pose.orientation.y = quat[1];
  target_pose.pose.orientation.z = quat[2];
  target_pose.pose.orientation.w = quat[3];

  bool check_transform_ok=true;
  geometry_msgs::PoseStamped transformed_pose = changePoseFrame(base_link_name_, target_pose, check_transform_ok);
  if (!check_transform_ok) {
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::OPERATIONAL_EXCEPTION;
    as_ptr_->publishFeedback(feedback_);
    ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                // publishing the result
    result_.success = false;
    result_.millis_passed = feedback_.millis_passed;
    result_.status = moveit_action_handlers::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
    ROS_WARN("[%s] Set Aborted from frameID ", action_name_.c_str());
    as_ptr_->setAborted(result_);
    return;
  }

  // set the pose target
  move_group_->setPoseTarget(transformed_pose);

  //*/////////// Define Motion Constraints
  // this will get and keep the current end effector orientation as constraint in the whole plan
  if (goal->constraint_mode == 1) {
    tf2_ros::Buffer br;
    br.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf2_listener(br);
    geometry_msgs::TransformStamped transform;
    try {
      transform = br.lookupTransform(base_link_name_, ee_link_name_, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::LookupException &e) {
      ROS_ERROR("[%s] %s", action_name_.c_str(), e.what());
    }

    moveit_msgs::OrientationConstraint ocm;

    ocm.link_name = ee_link_name_;
    ocm.header.frame_id = base_link_name_;
    ocm.orientation.x = transform.transform.rotation.x;
    ocm.orientation.y = transform.transform.rotation.y;
    ocm.orientation.z = transform.transform.rotation.z;
    ocm.orientation.w = transform.transform.rotation.w;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = .6;

    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints ori_constraints;
    ori_constraints.orientation_constraints.push_back(ocm);
    move_group_->setPathConstraints(ori_constraints);
  }
  //*/////////// END Define Constraint

  // We make the Plan in a different thread so we can publish feedback
  plannig_thread_done_ = false;
  boost::thread planning_thread(&MoveToPoseActionServer::moveit_planning, this);
  while (!plannig_thread_done_) {
    ros::Duration(0.001).sleep();
    // Planning ...
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::PLANNING;
    as_ptr_->publishFeedback(feedback_);

    // Check if we Preempt the goal
    if (as_ptr_->isPreemptRequested() || !ros::ok()) {

      move_group_->stop();
      move_group_->clearPathConstraints();
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

  // Planning Has Failed
  if (!planning_succeeded_) {

    move_group_->clearPathConstraints();
    ROS_ERROR("[%s] Planning Pose goal FAILED", action_name_.c_str());
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    as_ptr_->publishFeedback(feedback_);
    ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                // publishing the result
    ROS_ERROR("[%s]: Aborted. No plan found.", action_name_.c_str());
    result_.success = false;
    result_.status = moveit_action_handlers::ActionResultStatusConstants::PLANNING_FAILED;
    result_.millis_passed = feedback_.millis_passed;
    as_ptr_->setAborted(result_);
    return;
  }
  // Planning Succeded
  ROS_INFO("[%s] Planning Pose goal Succeded", action_name_.c_str());
  ROS_INFO("Visualizing plan to target: %s", planning_succeeded_ ? "SUCCEEDED" : "FAILED");

  // We make the Execution in a different thread so we can publish feedback
  execution_thread_done_ = false;
  boost::thread execution_thread(&MoveToPoseActionServer::moveit_execution, this);
  while (!execution_thread_done_) {
    ros::Duration(0.001).sleep();
    feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
    feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::EXECUTING;
    as_ptr_->publishFeedback(feedback_);
    // Check if we Preempt the goal
    if (as_ptr_->isPreemptRequested() || !ros::ok()) {
      move_group_->stop();
      if (goal->constraint_mode)
        move_group_->clearPathConstraints();
      feedback_.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
      feedback_.status = moveit_action_handlers::ActionFeedbackStatusConstants::CANCELLING;
      as_ptr_->publishFeedback(feedback_);
      ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                  // publishing the result
      ROS_WARN("[%s]: Preempted", action_name_.c_str());
      result_.success = false;
      result_.status = moveit_action_handlers::ActionResultStatusConstants::CANCELLED;
      result_.millis_passed = feedback_.millis_passed;
      as_ptr_->setPreempted(result_);
      return;
    }
  }

  // If Execution failed
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
  move_group_->clearPathConstraints();
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

void MoveToPoseActionServer::moveit_planning() {
  move_group_->setStartStateToCurrentState();
  planning_succeeded_ = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  plannig_thread_done_ = true;
}

void MoveToPoseActionServer::moveit_execution() {
  execution_succeeded_ = (move_group_->execute(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  execution_thread_done_ = true;
}

geometry_msgs::PoseStamped MoveToPoseActionServer::changePoseFrame(const std::string &base_frame,
                                                                   const geometry_msgs::PoseStamped &goal_pose,
                                                                   bool &check_transform_ok) {
  tf2_ros::Buffer br;
  br.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf2_listener(br);
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseStamped transformed_pose;

  try {
    transform = br.lookupTransform(base_frame, goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(goal_pose, transformed_pose, transform);
    ROS_INFO_STREAM("Change transform pose to..   " << transformed_pose);
  } catch (tf2::LookupException &e) {
    ROS_ERROR("[%s] %s", action_name_.c_str(), e.what());
    check_transform_ok = false;
  }
  return transformed_pose;
}
