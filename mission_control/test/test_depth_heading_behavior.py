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
from mission_control.msg import DepthHeading
from pose_estimator.msg import CorrectedData
from geometry_msgs.msg import Vector3
from mission_interface import MissionInterface
from mission_interface import wait_for

class TestDepthHeadingBehavior(unittest.TestCase):
    """ 
        The test loads and execute a depth heading mission.
        -   Load the mission depth_heading_mission_test.xml
        -   Execute the mission
        -   Check if the behavior publishes the goal
        -   Simulate PoseEstimator data to finish the behavior
        -   Test if the mission is SUCCESS
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_depth_heading_behavior')

    def setUp(self):
        self.depth_heading_goal = DepthHeading()
        self.mission = MissionInterface()
        
        self.depth_heading_msg = rospy.Subscriber(
            '/mngr/depth_heading',
            DepthHeading,
            self.depth_heading_goal_callback)

        self.simulated_pose_estimator_pub = rospy.Publisher(
            '/pose/corrected_data',
            CorrectedData,
            queue_size=1)

    def depth_heading_goal_callback(self, msg):
        self.depth_heading_goal = msg

    def test_mission_with_depth_heading_behavior(self):
        self.mission.load_mission('depth_heading_mission_test.xml')
        self.mission.execute_mission()
        
        self.mission.read_behavior_parameters('DepthHeadingBehavior')
        depth = self.mission.get_behavior_parameter('depth')
        heading = self.mission.get_behavior_parameter('heading')
        speed_knots = self.mission.get_behavior_parameter('speed_knots')
        enable_mask = 0

        #Calculate the mask
        def get_mask(value, mask):
            return mask if value > 0 else 0
        enable_mask |= get_mask(depth, DepthHeading.DEPTH_ENA)
        enable_mask |= get_mask(heading, DepthHeading.HEADING_ENA)
        enable_mask |= get_mask(speed_knots, DepthHeading.SPEED_KNOTS_ENA)
        
        def depth_heading_goals_are_set():
            return (self.depth_heading_goal.depth == depth and
                    self.depth_heading_goal.heading == heading and
                    self.depth_heading_goal.speed_knots == speed_knots and
                    self.depth_heading_goal.ena_mask == enable_mask)
        self.assertTrue(wait_for(depth_heading_goals_are_set),
                        msg='Mission control must publish goals')

        #send data to finish the mission
        rpy_data = Vector3()
        pose_estimator_corrected_data = CorrectedData()
        rpy_data.x = 0.0
        rpy_data.y = 0.0
        rpy_data.z = 2.0
        pose_estimator_corrected_data.rpy_ang = rpy_data
        self.simulated_pose_estimator_pub.publish(
            pose_estimator_corrected_data)

        def success_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report SUCCESS')

if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_test_depth_heading_behavior',
                   TestDepthHeadingBehavior)
