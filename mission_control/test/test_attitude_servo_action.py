#!/usr/bin/env python
"""
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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


try:
    from math import isclose
except ImportError:
    def isclose(a, b, rel_tol=1e-9):
        return abs(a - b) < (rel_tol * max(abs(a), abs(b)))


class TestAttitudeServoAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_attitude_servo')

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

    def test_mission_with_attitude_servo_action(self):
        """
        This test:
        -  Loads a mission with a single AttitudeServo action.
        -  Executes the mission just loaded.
        -  Waits until the action publishes an AttitudeServo setpoint.
        -  Simulates state updates for the action to complete.
        -  Waits until the mission is COMPLETE.
        """
        self.mission.load_mission('attitude_servo_mission_test.xml')
        self.mission.execute_mission()

        self.mission.read_behavior_parameters('AttitudeServo')
        roll = float(self.mission.get_behavior_parameter('roll'))
        pitch = float(self.mission.get_behavior_parameter('pitch'))
        pitch_units = self.mission.get_behavior_parameter('pitch-units')
        self.assertEqual('degrees', pitch_units)
        pitch = math.radians(float(pitch))
        yaw = float(self.mission.get_behavior_parameter('yaw'))
        speed_knots = float(self.mission.get_behavior_parameter('speed_knots'))

        # Calculate the mask
        enable_mask = 0
        if roll is not None:
            enable_mask |= AttitudeServo.ROLL_ENA
        if pitch is not None:
            enable_mask |= AttitudeServo.PITCH_ENA
        if yaw is not None:
            enable_mask |= AttitudeServo.YAW_ENA
        if speed_knots is not None:
            enable_mask |= AttitudeServo.SPEED_KNOTS_ENA

        def attitude_servo_goals_are_set():
            return (isclose(self.attitude_servo_goal.roll, roll) and
                    isclose(self.attitude_servo_goal.pitch, pitch, rel_tol=1e-6) and
                    isclose(self.attitude_servo_goal.yaw, yaw) and
                    isclose(self.attitude_servo_goal.speed_knots, speed_knots) and
                    self.attitude_servo_goal.ena_mask == enable_mask)

        self.assertTrue(wait_for(attitude_servo_goals_are_set),
                        msg='Mission control must publish goals')

        # send data to finish the mission
        msg = StateStamped()
        msg.state.manoeuvring.pose.mean.orientation.x = roll
        msg.state.manoeuvring.pose.mean.orientation.y = pitch
        msg.state.manoeuvring.pose.mean.orientation.z = yaw
        self.simulated_auv_interface_data_pub.publish(msg)

        def success_mission_status_is_reported():
            return (ReportExecuteMissionState.ABORTING not in self.mission.execute_mission_state and
                    ReportExecuteMissionState.COMPLETE in self.mission.execute_mission_state)
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report only COMPLETE')

if __name__ == '__main__':
    rostest.rosrun('mission_control', 'test_attitude_servo_action', TestAttitudeServoAction)
