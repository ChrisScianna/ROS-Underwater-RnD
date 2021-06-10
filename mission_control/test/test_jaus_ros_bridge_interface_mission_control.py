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
from __future__ import print_function

import sys
import os
import unittest
import rosnode
import rospy
import rostest
from mission_control.msg import LoadMission
from mission_control.msg import ExecuteMission
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import ReportLoadMissionState
from mission_control.msg import AbortMission
from mission_control.msg import QueryMissions
from mission_control.msg import RemoveMissions
from mission_control.msg import ReportMissions
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestJausRosBridgeInterface(unittest.TestCase):
    """
        Simulate messages sent by the jaus ros bridge:
        -   Load Mission
        -   Execute Mission
        -   Query Mission
        -   Abort Mission
        -   Remove Mission 
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_mission_control_interface_jaus_ros_bridge')

    def setUp(self):
        self.report_mission = ReportMissions()
        self.mission = MissionInterface()
        self.mission_load_state = None
        self.mission_to_load = LoadMission()

        # Subscribers
        self.load_state_sub = rospy.Subscriber(
            '/mission_control_node/report_mission_load_state',
            ReportLoadMissionState,
            self.callback_mission_load_state)

        self.simulated_mission_control_load_mission_pub = rospy.Publisher(
            '/mission_control_node/load_mission',
            LoadMission,
            latch=True,
            queue_size=1)

        self.report_missions_sub = rospy.Subscriber(
            '/mission_control_node/report_missions',
            ReportMissions,
            self.callback_report_mission)

        #   Publishers
        self.simulated_query_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/query_missions',
            QueryMissions,
            latch=True,
            queue_size=1)

        self.simulated_remove_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/remove_missions',
            RemoveMissions,
            latch=True,
            queue_size=1)

        self.simulated_abort_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/abort_mission',
            AbortMission,
            latch=True,
            queue_size=1)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg

    def callback_report_mission(self, msg):
        self.report_mission = msg

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state

    def test_jaus_ros_bridge_interface(self):
        # Test if the mission control reports FAILED if cannot load a mission
        self.assertEqual(self.mission.load_mission('NO_MISSION'), ReportLoadMissionState.FAILED)

        # A valid mission is sent to the mission control
        # Execute Mission and check if the mission control reports status
        self.mission.load_mission('waypoint_mission_test.xml')
        self.mission.execute_mission()

        # Wait for the mission to report EXECUTING
        def executing_mission_status_is_reported():
            return ReportExecuteMissionState.EXECUTING in self.mission.execute_mission_state
        self.assertTrue(
            wait_for(executing_mission_status_is_reported),
            msg='Mission control must report EXECUTING')

        # Abort mission and wait for it to wrap up
        abortMission = AbortMission()
        abortMission.mission_id = 1
        self.simulated_abort_mission_msg_pub.publish(abortMission)

        def aborting_mission_status_is_reported():
            return ReportExecuteMissionState.ABORTING in self.mission.execute_mission_state
        self.assertTrue(
            wait_for(aborting_mission_status_is_reported),
            msg='Mission control must report ABORTING')

        # Query mission and test response
        self.simulated_query_mission_msg_pub.publish(QueryMissions())

        def query_mission():
            return len(self.report_mission.missions) > 0
        self.assertTrue(wait_for(query_mission))
        self.assertEqual(
            self.report_mission.missions[0].mission_description, 'Test Mission')

        # Remove all missions and test if they have been removed.
        self.report_mission = ReportMissions()
        self.simulated_remove_mission_msg_pub.publish(RemoveMissions())
        self.simulated_query_mission_msg_pub.publish(QueryMissions())

        def query_after_remove_mission():
            return len(self.report_mission.missions) == 0
        self.assertTrue(wait_for(query_after_remove_mission))


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_interface_jaus_ros_bridge',
                   TestJausRosBridgeInterface)
