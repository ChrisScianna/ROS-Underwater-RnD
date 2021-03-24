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
import auv_interfaces

from mission_interface import MissionInterface


class TestGoToWaypoint(unittest.TestCase):
    """
        The test simulates messages sent by the jaus ros bridge
        and execute a waypoint behavior
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_go_to_waypoint')

    def setUp(self):
        self.waypoint = None
        self._waypoint_sub = rospy.Subscriber(
            '/mngr/waypoint', mission_control.msg.Waypoint,
            self._waypoint_callback)

    def _waypoint_goal(self, msg):
        self.waypoint = msg

    def test_mission_that_goes_to_waypoint(self):
        self.assertTrue(self.mission.load_mission(
            "go_to_waypoint_mission_test.xml"))

        self.mission.execute_mission()

        self.mission.read_behavior_parameters('MoveWithFixedRudder')
        depth = self.mission.get_behavior_parameter('depth')
        rudder = self.mission.get_behavior_parameter('rudder')
        speed_knots = self.mission.get_behavior_parameter('speed_knots')

        def waypoint_goal_is_set():
            
        self.assertTrue(wait_for(waypoint_goal_is_set))

        # TEST - Waypoint has published the goal
        while (not rospy.is_shutdown() and self.waypoint_goal_flag == False):
            rospy.sleep(0.1)
        self.assertEqual(self.waypoint_goal, [3.0, 4.0, 1.0, 0.0, 6.0, 23])
        rospy.sleep(4)

        # TEST - Pusblish Pose Estimator Data
        pose_estimator_corrected_data = CorrectedData()
        rospy.loginfo(pose_estimator_corrected_data)
        simulated_pose_estimator_pub.publish(pose_estimator_corrected_data)

        # TODO
        # test if the behavior has ended.


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'test_go_to_waypoint', TestGoToWaypoint)
