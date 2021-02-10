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
import rosnode
import rospy
import rostest
from mission_control.msg import LoadMission
from mission_control.msg import ExecuteMission
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import ReportLoadMissionState
from health_monitor.msg import ReportFault


def wait_for(predicate, period=1):
    while not rospy.is_shutdown():
        result = predicate()
        if result:
            return result
        rospy.sleep(period)
    return predicate()


class TestMissionControlAbortsWhenHealthMonitorReportsFault(unittest.TestCase):
    """ 
        Simulate error message sent by health monitor and checks 
        if the mission control publishes the abort status.
        The steps int the involved are:
            1)  Load a mission
            2)  Execute the Mission
            3)  Simulate an error sent by health monitor
            4)  check if the mission control abort the mission
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_mission_control_aborts_procedure')

    def setUp(self):
        self.dir_path = os.path.dirname(
            os.path.abspath(__file__)) + '/test_files/'
        self.mission_to_load = LoadMission()
        self.mission_to_load.mission_file_full_path = self.dir_path + \
            "test_missions/mission.xml"
        self.mission_to_execute = ExecuteMission()
        self.mission_to_execute.mission_id = 1
        self.simulate_error_code = ReportFault()
        self.simulate_error_code.fault_id = ReportFault.PAYLOAD_ERROR
        self.mission_load_state = ReportLoadMissionState()
        self.report_execute_mission = ReportExecuteMissionState()

        # Subscribers
        self.exec_state_sub = rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                                               ReportExecuteMissionState, self.callback_mission_execute_state)

        self.load_state_sub = rospy.Subscriber('/mission_control_node/report_mission_load_state',
                                               ReportLoadMissionState, self.callback_mission_load_state)

        #   Publisher

        self.simulated_mission_control_load_mission_pub = rospy.Publisher('/mission_control_node/load_mission',
                                                                          LoadMission, latch=True, queue_size=1)

        self.simulated_mission_control_execute_mission_pub = rospy.Publisher(
            '/mission_control_node/execute_mission', ExecuteMission, latch=True, queue_size=1)

        self.simulated_health_monitor_pub = rospy.Publisher('/health_monitor/report_fault',
                                                            ReportFault, queue_size=1)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state
        self.mission_load_state_flag = True

    def test_mission_control_aborts_if_health_monitor_reports_fault(self):

        # Load Mission
        self.simulated_mission_control_load_mission_pub.publish(
            self.mission_to_load)

        def test_load_valid_mission():
            return self.mission_load_state == ReportLoadMissionState.SUCCESS
        self.assertTrue(wait_for(test_load_valid_mission),
                        msg='Error Loading mission')

        # Execute Mission and check if the mission control reports status
        self.simulated_mission_control_execute_mission_pub.publish(
            self.mission_to_execute)

        def test_success_mission_status_is_reported():
            return self.report_execute_mission.execute_mission_state == ReportExecuteMissionState.EXECUTING
        self.assertTrue(wait_for(test_success_mission_status_is_reported),
                        msg='Mission control must report SUCCESS')

        # Simulate the health monitor publishing the fault code
        self.simulated_health_monitor_pub.publish(self.simulate_error_code)

        def test_abort_mission_status_is_reported():
            return self.report_execute_mission.execute_mission_state == ReportExecuteMissionState.PAUSED
        self.assertTrue(wait_for(test_abort_mission_status_is_reported),
                        msg='Mission control must report STOP/ABORTING')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_aborts_when_health_monitor_reports_fault',
                   TestMissionControlAbortsWhenHealthMonitorReportsFault)
