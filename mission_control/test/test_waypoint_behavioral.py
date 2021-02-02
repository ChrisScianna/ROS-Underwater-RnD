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
from mission_control.msg import LoadMission
from mission_control.msg import ExecuteMission
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import ReportLoadMissionState
from mission_control.msg import AbortMission
from mission_control.msg import QueryMissions
from mission_control.msg import RemoveMissions
from mission_control.msg import ReportMissions


class TestWaypointBehavioral(unittest.TestCase):
    """ 
        The test simulates messages sent by the jaus ros bridge
        and execute a waypoint behavioral
    """

    rospy.init_node('test_waypoint_behavioral')
    mission_execute_state = ReportExecuteMissionState.PAUSED
    mission_load_state = False
    dir_path = os.path.dirname(os.path.abspath(__file__)) + '/test_files/'

    mission_load_state_flag = False
    report_mission_flag = False
    report_mission = ReportMissions()

    def callback_mission_execute_state(self, msg):
        self.report_mission = msg
        print(msg)

    def callback_report_mission(self, msg):
        self.report_mission = msg
        self.report_mission_flag = True

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state
        self.mission_load_state_flag = True

    def test_mission_with_waypoint_behavioral(self):
        rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                         ReportExecuteMissionState, self.callback_mission_execute_state)

        rospy.Subscriber('/mission_control_node/report_mission_load_state',
                         ReportLoadMissionState, self.callback_mission_load_state)

        rospy.Subscriber('/mission_control_node/report_missions',
                         ReportMissions, self.callback_report_mission)

        simulated_mission_control_load_mission_pub = rospy.Publisher('/mission_control_node/load_mission',
                                                                     LoadMission, latch=True, queue_size=1)

        simulated_mission_control_execute_mission_pub = rospy.Publisher(
            '/mission_control_node/execute_mission', ExecuteMission, latch=True, queue_size=1)

        simulated_abort_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/abort_mission', AbortMission, latch=True, queue_size=1)

        simulated_query_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/query_missions', QueryMissions, latch=True, queue_size=1)

        simulated_remove_mission_msg_pub = rospy.Publisher(
            '/mission_control_node/remove_missions', RemoveMissions, latch=True, queue_size=1)
        
        simulated_mission_manager_execute_mission_pub = rospy.Publisher('/mission_control_node/execute_mission', 
            ExecuteMission, queue_size=1)

        # Load mission with waypoint behavioral
        self.mission_load_state_flag = False
        mission_to_load = LoadMission()
        mission_to_load.mission_file_full_path = self.dir_path + "test_missions/waypoint_mission_test.xml"
        simulated_mission_control_load_mission_pub.publish(mission_to_load)
        while not rospy.is_shutdown() and not self.mission_load_state_flag:
            rospy.sleep(0.1)
        self.assertEqual(self.mission_load_state, ReportLoadMissionState.SUCCESS)

        # TEST - Query Mission
        mission_to_query = QueryMissions()
        simulated_query_mission_msg_pub.publish(mission_to_query)
        while (not rospy.is_shutdown() and self.report_mission_flag == False):
            rospy.sleep(0.1)
        self.assertEqual(self.report_mission.missions[0].mission_description, "mission test")

        # TEST - Execute Mission
        execute_mission_command = ExecuteMission()
        execute_mission_command.mission_id = 1
        simulated_mission_manager_execute_mission_pub.publish(execute_mission_command)
        rospy.sleep(1)
        
        # TODO
        # Publish data that allows waypoint to finish.
        # test if the mission has ended.

if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_test_waypoint_behavioral',
                   TestWaypointBehavioral)
