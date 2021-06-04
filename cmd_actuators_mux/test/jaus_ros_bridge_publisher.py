#!/usr/bin/env python

import sys
import os
import unittest
import math
import rosnode
import rospy
import rostest
from fin_control.msg import SetAngles

if __name__ == '__main__':
    rospy.init_node('test_jrb_publisher')
    msg = SetAngles()
    simulated_autopilot = rospy.Publisher(
        '/CmdActuatorMuxNodelet/input/jausRosBridge/set_angles',
        SetAngles,
        queue_size=1)
    rate = rospy.Rate(5) # 10hz
    msg.f1_angle_in_radians = 2.
    msg.f2_angle_in_radians = 2.
    msg.f3_angle_in_radians = 2.
    msg.f4_angle_in_radians = 2.

    while not rospy.is_shutdown():
        simulated_autopilot.publish(msg)
        rate.sleep()
