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
from mission_control.msg import ReportMissions
from mission_control.msg import FixedRudder


def wait_for(predicate, period=1):
    while not rospy.is_shutdown():
        result = predicate()
        if result:
            return result
        rospy.sleep(period)
    return predicate()


class TestFixedRudderBehavior(unittest.TestCase):
    """ 
        The test loads and execute a fixed rudder mission.
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_fixed_rudder_behavioral')

    def setUp(self):
        mission_execute_state = ReportExecuteMissionState.PAUSED
        mission_load_state = False
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) + '/test_files/'
        mission_load_state_flag = False
        report_mission_flag = False
        report_mission = ReportMissions()
        self.mission_load_state = None
        self.report_mission = None
        self.report_execute_mission = None
        self.fixed_rudder_goal = []

        # Subscribers
        self.exec_state_sub = rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                                               ReportExecuteMissionState, self.callback_mission_execute_state)

        self.load_state_sub = rospy.Subscriber('/mission_control_node/report_mission_load_state',
                                               ReportLoadMissionState, self.callback_mission_load_state)

        self.report_missions_sub = rospy.Subscriber('/mission_control_node/report_missions',
                                                    ReportMissions, self.callback_report_mission)

        self.fixed_rudder_msg = rospy.Subscriber('/mngr/fixed_rudder',
                                                 FixedRudder, self.callback_publish_fixed_rudder_goal)

        # Publishers
        self.simulated_mission_control_load_mission_pub = rospy.Publisher('/mission_control_node/load_mission',
                                                                          LoadMission, latch=True, queue_size=1)

        self.simulated_mission_control_execute_mission_pub = rospy.Publisher(
            '/mission_control_node/execute_mission', ExecuteMission, latch=True, queue_size=1)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg
        rospy.loginfo(msg)

    def callback_report_mission(self, msg):
        self.report_mission = msg

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state

    def callback_publish_fixed_rudder_goal(self, msg):
        self.fixed_rudder_goal = [msg.depth,
                                  msg.rudder,
                                  msg.speed_knots,
                                  msg.ena_mask]

    def test_mission_with_fixed_rudder_behavioral(self):

        self.mission_load_state_flag = False
        mission_to_load = LoadMission()
        mission_to_load.mission_file_full_path = self.dir_path + "test_missions/fixed_rudder_mission_test.xml"
        self.simulated_mission_control_load_mission_pub.publish(mission_to_load)

        def load_mission():
            return self.mission_load_state == ReportLoadMissionState.SUCCESS
        self.assertTrue(wait_for(load_mission),
                        msg='Mission control must report SUCCESS')
        rospy.loginfo("Mission Loaded")

        mission_to_execute = ExecuteMission()
        mission_to_execute.mission_id = 1
        self.simulated_mission_control_execute_mission_pub.publish(
            mission_to_execute)
        rospy.loginfo("execute msg")

        def fixed_rudder_goals_are_setted():
            return self.fixed_rudder_goal == [1.0, 2.0, 3.0, 7]
        self.assertTrue(wait_for(fixed_rudder_goals_are_setted),
                        msg='Mission control must publish goals')

        def success_mission_status_is_reported():
            return self.report_execute_mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported), 
            msg='Mission control must report SUCCESS')

if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_test_fixed_rudder_behavioral',
                   TestFixedRudderBehavior)