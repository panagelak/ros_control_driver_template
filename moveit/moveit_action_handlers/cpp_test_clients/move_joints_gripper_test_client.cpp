
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_action_handlers/MoveToJointsMoveItAction.h>
#include <moveit_action_handlers/MoveToJointsMoveItGoal.h>
#include <moveit_action_handlers/PropertyValuePair.h>
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_joints_gripper_client_cpp_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<moveit_action_handlers::MoveToJointsMoveItAction> action_client_gripper(
      "move_tojoints_action_server/gripper/action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  action_client_gripper.waitForServer(); // will wait for infinite time

  // Creates a goal to send to the action server arm.
  moveit_action_handlers::MoveToJointsMoveItGoal goal;
  goal.endEffectorVelocity = 0.5;
  goal.endEffectorAcceleration = 0.5;
  goal.timeoutSeconds = 20;
  //  joint names
  std::vector<std::string> joint_names = {"finger1_joint", "finger2_joint"};

  // gripper joint values close
  std::vector<float> joint_values = {0.02, 0.02};
  // gripper joint values open
  // std::vector<float> joint_values = {0.0, 0.0};
  for (size_t i = 0; i < joint_names.size(); i++) {
    moveit_action_handlers::PropertyValuePair joint_pair;
    joint_pair.name = joint_names[i];
    joint_pair.value = joint_values[i];
    // push back to goal the joint pair
    goal.joint_pairs.push_back(joint_pair);
  }

  // send goal
  action_client_gripper.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = action_client_gripper.waitForResult(ros::Duration(20.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = action_client_gripper.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else
    ROS_INFO("Action did not finish before the time out.");

  // exit
  return 0;
}