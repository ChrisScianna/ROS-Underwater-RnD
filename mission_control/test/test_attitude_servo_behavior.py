#!/usr/bin/env python
"""
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
"""
import sys
import os
import unittest
import math
import rosnode
import rospy
import rostest
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import AttitudeServo
from auv_interfaces.msg import StateStamped
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestAttitudeServoBehavior(unittest.TestCase):
    """
        The test loads and execute a attitude servo mission.
        -   Load the mission attitude_servo_mission_test.xml
        -   Execute the mission
        -   Check if the behavior publishes the goal
        -   Simulate auv_inteface data to finish the behavior
        -   Test if the mission is SUCCESS
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_attitude_servo_behavioral')

    def setUp(self):
        self.attitude_servo_goal = AttitudeServo()
        self.mission = MissionInterface()

        # Subscribers
        self.attitude_servo_msg = rospy.Subscriber(
            '/mngr/attitude_servo',
            AttitudeServo,
            self.attitude_servo_goal_callback)

        self.simulated_auv_interface_data_pub = rospy.Publisher(
            '/state',
            StateStamped,
            queue_size=1)

    def attitude_servo_goal_callback(self, msg):
        self.attitude_servo_goal = msg

    def test_mission_with_attitude_servo_behavior(self):
        self.mission.load_mission('attitude_servo_mission_test.xml')
        self.mission.execute_mission()

        self.mission.read_behavior_parameters('AttitudeServoBehavior')
        roll = self.mission.get_behavior_parameter('roll')
        pitch = self.mission.get_behavior_parameter('pitch')
        yaw = self.mission.get_behavior_parameter('yaw')
        speed_knots = self.mission.get_behavior_parameter('speed_knots')
        enable_mask = 0

        # Calculate the mask
        def get_mask(value, mask):
            return mask if value > 0 else 0
        enable_mask |= get_mask(roll, AttitudeServo.ROLL_ENA)
        enable_mask |= get_mask(pitch, AttitudeServo.PITCH_ENA)
        enable_mask |= get_mask(yaw, AttitudeServo.YAW_ENA)
        enable_mask |= get_mask(speed_knots, AttitudeServo.SPEED_KNOTS_ENA)

        # Check if the behavior publishes the goal
        def attitude_servo_goals_are_set():
            return (self.attitude_servo_goal.roll == roll and
                    self.attitude_servo_goal.pitch == pitch and
                    self.attitude_servo_goal.yaw == yaw and
                    self.attitude_servo_goal.speed_knots == speed_knots and
                    self.attitude_servo_goal.ena_mask == enable_mask)
        self.assertTrue(wait_for(attitude_servo_goals_are_set),
                        msg='Mission control must publish goals')

        # send data to finish the mission
        auv_interface_data = StateStamped()
        auv_interface_data.state.manoeuvring.pose.mean.orientation.x = 1.0
        auv_interface_data.state.manoeuvring.pose.mean.orientation.y = 1.0
        auv_interface_data.state.manoeuvring.pose.mean.orientation.z = 1.0
        self.simulated_auv_interface_data_pub.publish(auv_interface_data)

        def success_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report SUCCESS')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'test_mission_with_attitude_servo_behavior',
                   TestAttitudeServoBehavior)
