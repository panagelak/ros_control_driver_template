
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_action_handlers/MoveToPoseMoveItAction.h>
#include <moveit_action_handlers/MoveToPoseMoveItGoal.h>
#include <moveit_action_handlers/PoseStamped.h>
#include <ros/ros.h>
#include <string>


int main(int argc, char **argv) {
  ros::init(argc, argv, "test_topose_arm_client_cpp_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<moveit_action_handlers::MoveToPoseMoveItAction> action_client_arm(
      "move_topose_action_server/arm/action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  action_client_arm.waitForServer(); // will wait for infinite time

  // Creates a goal to send to the action server arm.
  moveit_action_handlers::MoveToPoseMoveItGoal goal;
  goal.constraint_mode = 0;
  goal.endEffectorVelocity = 0.5;
  goal.endEffectorAcceleration = 0.5;
  goal.timeoutSeconds = 20;

  // euler pose
  moveit_action_handlers::PoseStamped target_pose;
  // relative to end effector frame
  target_pose.header.frame_id = "ee_base";
  target_pose.pose.position.x = 0.03;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.03;
  target_pose.pose.orientation.roll = 0.0;
  target_pose.pose.orientation.pitch = 1.0;
  target_pose.pose.orientation.yaw = 0.0;

  // pass the euler pose to goal pose
  goal.target_pose = target_pose;

  // send goal
  action_client_arm.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = action_client_arm.waitForResult(ros::Duration(20.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = action_client_arm.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
    ROS_INFO("Action did not finish before the time out.");

  // exit
  return 0;
}