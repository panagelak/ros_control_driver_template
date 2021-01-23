#! /usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib

from moveit_action_handlers.msg import MoveToPoseMoveItAction
from moveit_action_handlers.msg import MoveToPoseMoveItGoal
# Euler pose
from moveit_action_handlers.msg import PoseStamped

def moveit_topose_client():
    client = actionlib
    client = actionlib.simple_action_client.SimpleActionClient(
        'move_topose_action_server/arm/action', MoveToPoseMoveItAction)

    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveToPoseMoveItGoal()
    # Define Goal
    goal.constraint_mode = 0
    goal.endEffectorVelocity = 0.5
    goal.endEffectorAcceleration = 0.5
    goal.timeoutSeconds = 10
    # euler pose
    target_pose = PoseStamped()
    # relative to end effector frame
    target_pose.header.frame_id = "ee_base"
    target_pose.pose.position.x = 0.03
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.03
    target_pose.pose.orientation.roll = 0.0
    target_pose.pose.orientation.pitch = 1.0
    target_pose.pose.orientation.yaw = 0.0
    goal.target_pose = target_pose

    # send goal
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':

    rospy.init_node('test_pose_client_py_node')
    result = moveit_topose_client()