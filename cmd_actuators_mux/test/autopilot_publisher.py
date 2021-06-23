#!/usr/bin/env python

import sys
import os
import unittest
import math
import rosnode
import rospy
import rostest
from fin_control.msg import SetAngles
from thruster_control.msg import SetRPM

if __name__ == '__main__':
    rospy.init_node('test_autopilot_publisher')
    angleMsg = SetAngles()
    rpmMsg = SetRPM ()

    simulated_autopilot_angles = rospy.Publisher(
        '/input/autopilot/set_angles',
        SetAngles,
        queue_size=1)
    simulated_autopilot_RPM = rospy.Publisher(
        '/input/autopilot/set_rpm',
        SetRPM,
        queue_size=1)
    
    rate = rospy.Rate(2) # 10hz


    rpmMsg.commanded_rpms = 100
    angleMsg.f1_angle_in_radians = 1.
    angleMsg.f2_angle_in_radians = 1.
    angleMsg.f3_angle_in_radians = 1.
    angleMsg.f4_angle_in_radians = 1.

    while not rospy.is_shutdown():
        simulated_autopilot_angles.publish(angleMsg)
        simulated_autopilot_RPM.publish(rpmMsg)
        rate.sleep()
