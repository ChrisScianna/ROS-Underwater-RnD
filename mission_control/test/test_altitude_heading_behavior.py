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
from mission_control.msg import AltitudeHeading
from auv_interfaces.msg import StateStamped
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestAltitudeHeadingBehavior(unittest.TestCase):
    """
        The test loads and execute an altitude heading behavior.
        -   Load the mission altitude_heading_mission_test.xml
        -   Execute the mission
        -   Check if the behavior publishes the goal
        -   Simulate auv_inteface data to finish the behavior
        -   Test if the mission is COMPLETE
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_altitude_heading_behavior')

    def setUp(self):
        self.altitude_heading_goal = AltitudeHeading()
        self.mission = MissionInterface()

        self.altitude_heading_msg = rospy.Subscriber(
            '/mngr/altitude_heading',
            AltitudeHeading,
            self.altitude_heading_goal_callback)

        self.simulated_auv_interface_data_pub = rospy.Publisher(
            '/state',
            StateStamped,
            queue_size=1)

    def altitude_heading_goal_callback(self, msg):
        self.altitude_heading_goal = msg

    def test_mission_with_altitude_heading_behavior(self):
        self.mission.load_mission('altitude_heading_mission_test.xml')
        self.mission.execute_mission()
        self.mission.read_behavior_parameters('AltitudeHeadingBehavior')
        altitude = self.mission.get_behavior_parameter('altitude')
        heading = self.mission.get_behavior_parameter('heading')
        speed_knots = self.mission.get_behavior_parameter('speed_knots')

        # Calculate the mask
        enable_mask = 0
        if altitude is not None:
            enable_mask |= AltitudeHeading.ALTITUDE_ENA

        if heading is not None:
            enable_mask |= AltitudeHeading.HEADING_ENA

        if speed_knots is not None:
            enable_mask |= AltitudeHeading.SPEED_KNOTS_ENA

        def altitude_heading_goals_are_set():
            return ((altitude is None or self.altitude_heading_goal.altitude == float(altitude)) and
                    (heading is None or self.altitude_heading_goal.heading == float(heading)) and
                    (speed_knots is None or self.altitude_heading_goal.speed_knots == float(speed_knots)) and
                    self.altitude_heading_goal.ena_mask == enable_mask)

        self.assertTrue(wait_for(altitude_heading_goals_are_set),
                        msg='Mission control must publish goals')

        # send data to finish the mission
        auv_interface_data = StateStamped()
        auv_interface_data.state.manoeuvring.pose.mean.orientation.x = 0.0
        auv_interface_data.state.manoeuvring.pose.mean.orientation.y = 0.0
        auv_interface_data.state.manoeuvring.pose.mean.orientation.z = 2.0
        self.simulated_auv_interface_data_pub.publish(auv_interface_data)

        def success_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report COMPLETE')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_test_altitude_heading_behavior',
                   TestAltitudeHeadingBehavior)
