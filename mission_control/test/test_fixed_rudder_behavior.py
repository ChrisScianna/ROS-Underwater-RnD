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
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import FixedRudder
from load_execute_mission import LoadExecuteMission


class TestFixedRudderBehavior(unittest.TestCase):
    """
        The test:
        -   Load a fixed rudder mission.
        -   execute the mission
        -   Wait until the mission is complete (behavior_time)
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_fixed_rudder_behavior')

    def setUp(self):
        mission_execute_state = ReportExecuteMissionState.PAUSED
        self.report_execute_mission = None
        self.fixed_rudder_goal = []
        self.mission = LoadExecuteMission()
        self.fixed_rudder_goal = FixedRudder()

        # Subscribers
        self.exec_state_sub = rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                                               ReportExecuteMissionState, self.callback_mission_execute_state)

        self.fixed_rudder_msg = rospy.Subscriber('/mngr/fixed_rudder',
                                                 FixedRudder, self.callback_publish_fixed_rudder_goal)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg

    def callback_publish_fixed_rudder_goal(self, msg):
        self.fixed_rudder_goal = msg

    def test_mission_with_fixed_rudder_behavior(self):

        self.mission.load_mission("fixed_rudder_mission_test.xml")
        self.mission.execute_mission()

        # Check if the behavior publishes the goal
        def fixed_rudder_goals_are_setted():
            return (self.fixed_rudder_goal.depth == 1.0 and
                    self.fixed_rudder_goal.rudder == 2.0 and
                    self.fixed_rudder_goal.speed_knots == 3.0 and
                    self.fixed_rudder_goal.ena_mask == 7)
        self.assertTrue(self.mission.wait_for(fixed_rudder_goals_are_setted),
                        msg='Mission control must publish goals')

        # Wait for the mission to be complete
        def success_mission_status_is_reported():
            return self.report_execute_mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(self.mission.wait_for(success_mission_status_is_reported),
                        msg='Mission control must report COMPLETE')

if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_test_fixed_rudder_behavior',
                   TestFixedRudderBehavior)
