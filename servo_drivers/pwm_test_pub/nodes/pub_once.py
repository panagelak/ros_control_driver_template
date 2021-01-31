#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray


class pwmTestPythonPub(object):
    def __init__(self):
        # create publisher
        self._pub = rospy.Publisher(
            "/command", Int32MultiArray, queue_size=10, latch=True)
        # create the msg
        self._msg = Int32MultiArray()
        # fill the message with size -1
        for i in range(16):
            self._msg.data.append(-1)

        # create rate 50 hz
        self._freq = 50
        self._rate = rospy.Rate(self._freq)

        #self._msg.data[9] = 6000
        self._msg.data[10] = 6000
        #self._msg.data[11] = 6000
        #self._msg.data[12] = 6000
        #self._msg.data[13] = 6000
        #self._msg.data[14] = 6000
        #self._msg.data[15] = 6000
    def run(self):
        while not rospy.is_shutdown():
            # publish message
            self._pub.publish(self._msg)
            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pwm_test_python_pub_node')
    rospy.loginfo("pwm_test_python_pub_node started ...")
    # Instantiate object
    pwm_python_pub_obj = pwmTestPythonPub()
    # run object
    pwm_python_pub_obj.run()
    # spin the node
    rospy.spin()
