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


class LoadExecuteMission:
    """
        This library simulates the Jaus Ros Bridge commands:
        - Load Mission
        - Execute Mission
    """

    def __init__(self):
        mission_load_state_flag = False
        report_mission_flag = False
        report_mission = ReportMissions()
        self.mission_load_state = None

        self.report_execute_mission = None

        # Subscribers
        self.exec_state_sub = rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                                               ReportExecuteMissionState, self.callback_mission_execute_state)

        self.load_state_sub = rospy.Subscriber('/mission_control_node/report_mission_load_state',
                                               ReportLoadMissionState, self.callback_mission_load_state)

        # Publishers
        self.simulated_mission_control_load_mission_pub = rospy.Publisher('/mission_control_node/load_mission',
                                                                          LoadMission, latch=True, queue_size=1)

        self.simulated_mission_control_execute_mission_pub = rospy.Publisher(
            '/mission_control_node/execute_mission', ExecuteMission, latch=True, queue_size=1)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state

    def load_mission(self, mission_name):
        # Simulate Jaus Ros Bridge sending a Load Command and wait for
        # response that the mission was loaded correctly
        self.mission_load_state_flag = False
        mission_to_load = LoadMission()
        mission_to_load.mission_file_full_path = os.path.dirname(
            os.path.abspath(__file__)) + '/test_files/test_missions/' + mission_name
        self.simulated_mission_control_load_mission_pub.publish(
            mission_to_load)

        def load_mission_state():
            return self.mission_load_state == ReportLoadMissionState.SUCCESS
        self.wait_for(load_mission_state)

    def execute_mission(self, mission_id=1):
        # Simulate Jaus Ros Bridge sending an Execute Command
        mission_to_execute = ExecuteMission()
        mission_to_execute.mission_id = mission_id
        self.simulated_mission_control_execute_mission_pub.publish(
            mission_to_execute)

    def wait_for(self, predicate, period=1):
        while not rospy.is_shutdown():
            result = predicate()
            if result:
                return result
            rospy.sleep(period)
        return predicate()
