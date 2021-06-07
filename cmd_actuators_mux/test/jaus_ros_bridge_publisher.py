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
    rospy.init_node('test_jausRosBridge_publisher')
    angleMsg = SetAngles()
    rpmMsg = SetRPM ()

    simulated_jausRosBridge_angles = rospy.Publisher(
        '/input/jausRosBridge/set_angles',
        SetAngles,
        queue_size=1)
    simulated_jausRosBridge_RPM = rospy.Publisher(
        'input/jausRosBridge/set_rpm',
        SetRPM,
        queue_size=1)
    
    rate = rospy.Rate(2) # 10hz


    rpmMsg.commanded_rpms = 200
    angleMsg.f1_angle_in_radians = 2.
    angleMsg.f2_angle_in_radians = 2.
    angleMsg.f3_angle_in_radians = 2.
    angleMsg.f4_angle_in_radians = 2.

    while not rospy.is_shutdown():
        simulated_jausRosBridge_angles.publish(angleMsg)
        simulated_jausRosBridge_RPM.publish(rpmMsg)
        rate.sleep()