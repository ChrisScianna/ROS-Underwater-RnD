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
from mission_control.msg import DepthHeading
from auv_interfaces.msg import StateStamped
from mission_interface import MissionInterface
from mission_interface import wait_for

try:
    from math import isclose
except ImportError:
    def isclose(a, b, rel_tol=1e-9):
        return abs(a - b) < (rel_tol * max(abs(a), abs(b)))


class TestSetDepthHeadingAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_set_depth_heading')

    def setUp(self):
        self.depth_heading_goal = DepthHeading()
        self.mission = MissionInterface()

        self.depth_heading_msg = rospy.Subscriber(
            '/mngr/depth_heading',
            DepthHeading,
            self.depth_heading_goal_callback)

        self.simulated_auv_interface_data_pub = rospy.Publisher(
            '/state',
            StateStamped,
            queue_size=1)

    def depth_heading_goal_callback(self, msg):
        self.depth_heading_goal = msg

    def test_mission_with_set_depth_heading_action(self):
        """
        The test:
        -  Loads a mission with a single SetDepthHeading action.
        -  Executes the mission.
        -  Waits until the action publishes a DepthHeading setpoint.
        -  Simulates state updates for the action to complete.
        -  waits until the mission is COMPLETE.
        """
        self.mission.load_mission('depth_heading_mission_test.xml')
        self.mission.execute_mission()

        self.mission.read_behavior_parameters('SetDepthHeading')
        depth = float(self.mission.get_behavior_parameter('depth'))
        heading = float(self.mission.get_behavior_parameter('heading'))
        heading_units = self.mission.get_behavior_parameter('heading-units')
        self.assertEqual('degrees', heading_units)
        heading = math.radians(float(heading))
        speed_knots = float(self.mission.get_behavior_parameter('speed_knots'))

        # Calculate the mask
        enable_mask = DepthHeading.DEPTH_ENA
        enable_mask |= DepthHeading.HEADING_ENA
        enable_mask |= DepthHeading.SPEED_KNOTS_ENA

        def depth_heading_goals_are_set():
            return (isclose(self.depth_heading_goal.depth, depth) and
                    isclose(self.depth_heading_goal.heading, heading, rel_tol=1e-6) and
                    isclose(self.depth_heading_goal.speed_knots, speed_knots) and
                    self.depth_heading_goal.ena_mask == enable_mask)

        self.assertTrue(wait_for(depth_heading_goals_are_set),
                        msg='Mission control must publish goals')

        # send data to finish the mission
        auv_interface_data = StateStamped()
        auv_interface_data.state.manoeuvring.pose.mean.position.z = depth
        auv_interface_data.state.manoeuvring.pose.mean.orientation.z = heading
        self.simulated_auv_interface_data_pub.publish(auv_interface_data)

        def success_mission_status_is_reported():
            return (ReportExecuteMissionState.ABORTING not in self.mission.execute_mission_state and
                    ReportExecuteMissionState.COMPLETE in self.mission.execute_mission_state)
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report only COMPLETE')


if __name__ == '__main__':
    rostest.rosrun('mission_control', 'test_set_depth_heading_action', TestSetDepthHeadingAction)
