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
    rospy.init_node('test_autopilot_publisher')
    msg = SetAngles()
    simulated_autopilot = rospy.Publisher(
        '/input/autopilot/set_angles',
        SetAngles,
        queue_size=1)
    rate = rospy.Rate(2) # 10hz
    msg.f1_angle_in_radians = 1.
    msg.f2_angle_in_radians = 1.
    msg.f3_angle_in_radians = 1.
    msg.f4_angle_in_radians = 1.

    while not rospy.is_shutdown():
        simulated_autopilot.publish(msg)
        rate.sleep()
