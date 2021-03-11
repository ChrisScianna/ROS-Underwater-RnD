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
import rosnode
import rospy
import rostest
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import AttitudeServo
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestMissionControlAbortsWhenBehaviorReturnsTimeOutFailure(unittest.TestCase):
    """
        Test if the mission control aborts after the behavior returns Failure state.
        The failure is produced because the mission time is greater than the time_out
        mission behavior value
        Args:
            sys.argv[1] = Mission filename. It must be located in the test_mission folder.

        The steps involved are:
            1)  Load a mission
            2)  Execute the mission
            4)  Wait the mission control to publish a Failure state
            5)  Wait the mission control to publish to actuators
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_mission_control_aborts_procedure')

    def setUp(self):
        self.mission = MissionInterface()
        self.attitude_servo_aborting_goal = AttitudeServo()

        self.attitude_servo_msg = rospy.Subscriber(
            '/mngr/attitude_servo',
            AttitudeServo,
            self.attitude_servo_callback)

    def attitude_servo_callback(self, msg):
        self.attitude_servo_aborting_goal = msg

    def test_mission_control_aborts_if_health_monitor_reports_fault(self):
        self.mission.load_mission(sys.argv[1])
        self.mission.execute_mission()

        # Wait for the mission returns time_out Failure
        def aborting_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.ABORTING
        self.assertTrue(wait_for(aborting_mission_status_is_reported),
                        msg='Mission control must report ABORTING')

        # Check if the mission control publishes the attitude servo msg to
        # set the fins to surface and velocity to 0 RPM
        maxCtrlFinAngle = rospy.get_param('/fin_control/max_ctrl_fin_angle')

        def attitude_servo_aborting_goals_are_set():
            return (self.attitude_servo_aborting_goal.roll == 0.0 and
                    self.attitude_servo_aborting_goal.pitch == -maxCtrlFinAngle and
                    self.attitude_servo_aborting_goal.yaw == 0.0 and
                    self.attitude_servo_aborting_goal.speed_knots == 0.0 and
                    self.attitude_servo_aborting_goal.ena_mask == 15)
        self.assertTrue(wait_for(attitude_servo_aborting_goals_are_set),
                        msg='Mission control must publish goals')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_aborts_when_health_monitor_reports_fault',
                   TestMissionControlAbortsWhenBehaviorReturnsTimeOutFailure)
