#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from adafruit_servokit import ServoKit

class ServoKitRosDriver(object):
    def __init__(self):
        # create ros subscriber on command topic
        self._cmd_sub = rospy.Subscriber(
            "/command", Int32MultiArray, self.callback=)
        # create servo kit driver
        self.kit = ServoKit(channels=16)
        # Initialize pulse width range
        for i in xrange(16):
            self.kit.servo[i].set_pulse_width_range(1000, 2000)
        
    def callback(ros_msg):
        for ros_cmd in ros_msg.data:
            if ros_cmd == -1:
                continue
            kit.servo[0].angle = ros_cmd

if __name__ == '__main__':
    rospy.init_node('servokit_ros_driver')
    rospy.loginfo("pwm_test_python_pub_node started ...")
    # Instantiate object
    ServoKitRosDriver = ServoKitRosDriver()
    # spin the node
    rospy.spin()
