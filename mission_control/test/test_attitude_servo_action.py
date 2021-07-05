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
import math
import unittest

import rospy
import rostest

from auv_interfaces.msg import StateStamped
from mission_control.msg import AttitudeServo

from test_utilities import MissionControlInterface


class TestAttitudeServoAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('attitude_servo_test_node')

    def setUp(self):
        self.mission_control = MissionControlInterface()

    def test_mission_that_sets_attitude(self):
        """
        This test:
        -  Loads a mission with a single AttitudeServo action.
        -  Executes the mission just loaded.
        -  Waits until the action publishes an AttitudeServo setpoint.
        -  Simulates state updates for the action to complete.
        -  Waits until the mission is COMPLETE.
        """
        roll = 1.0
        pitch = 2.0
        yaw = 3.0
        speed_knots = 4.0
        ena_mask = AttitudeServo.ROLL_ENA
        ena_mask |= AttitudeServo.PITCH_ENA
        ena_mask |= AttitudeServo.YAW_ENA
        ena_mask |= AttitudeServo.SPEED_KNOTS_ENA

        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <Timeout msec="2000">
                <AttitudeServo
                    roll="{}"
                    pitch="{}"
                    pitch-units="degrees"
                    yaw="{}"
                    speed_knots="{}"
                    roll-tolerance="1.0"
                    pitch-tolerance="1.0"
                    yaw-tolerance="1.0"/>
              </Timeout>
            </BehaviorTree>
          </root>
        '''.format(roll, math.degrees(pitch), yaw, speed_knots)

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)

        msg = rospy.wait_for_message(
            '/mngr/attitude_servo', AttitudeServo)
        self.assertAlmostEqual(msg.roll, roll)
        self.assertAlmostEqual(msg.pitch, pitch)
        self.assertAlmostEqual(msg.yaw, yaw)
        self.assertAlmostEqual(msg.speed_knots, speed_knots)
        self.assertAlmostEqual(msg.ena_mask, ena_mask)

        msg = StateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.state.manoeuvring.pose.mean.orientation.x = roll
        msg.state.manoeuvring.pose.mean.orientation.y = pitch
        msg.state.manoeuvring.pose.mean.orientation.z = yaw
        rospy.Publisher(
            '/state', StateStamped,
            queue_size=1, latch=True
        ).publish(msg)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))


if __name__ == '__main__':
    rostest.rosrun(
        'mission_control',
        'test_attitude_servo_action',
        TestAttitudeServoAction)
