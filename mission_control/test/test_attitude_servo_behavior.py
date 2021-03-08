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
from mission_control.msg import AttitudeServo
from pose_estimator.msg import CorrectedData
from geometry_msgs.msg import Vector3
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestAttitudeServoBehavior(unittest.TestCase):
    """
        The test loads and execute a attitude servo mission.
        -   Load the mission attitude_servo_mission_test.xml
        -   Execute the mission
        -   Check if the behavior publishes the goal
        -   Simulate PoseEstimator data to finish the behavior
        -   Test if the mission is SUCCESS
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_attitude_servo_behavioral')

    def setUp(self):
        self.attitude_servo_goal = AttitudeServo()
        self.mission = MissionInterface()

        # Subscribers
        self.attitude_servo_msg = rospy.Subscriber('/mngr/attitude_servo',
                                                   AttitudeServo, self.callback_publish_attitude_servo_goal)

        self.simulated_pose_estimator_pub = rospy.Publisher('/pose/corrected_data',
                                                            CorrectedData, queue_size=1)

    def callback_publish_attitude_servo_goal(self, msg):
        self.attitude_servo_goal = msg
        rospy.loginfo(msg)

    def test_mission_with_attitude_servo_behavioral(self):
        self.mission.load_mission("attitude_servo_mission_test.xml")
        self.mission.execute_mission()
        rospy.loginfo("Ejecutando")

        self.mission.read_behavior_parameters('AttitudeServoBehavior')
        roll = self.mission.get_behavior_parameter('roll')
        pitch = self.mission.get_behavior_parameter('pitch')
        yaw = self.mission.get_behavior_parameter('yaw')
        speed_knots = self.mission.get_behavior_parameter('speed_knots')

        # Check if the behavior publishes the goal
        def attitude_servo_goals_are_set():
            return (self.attitude_servo_goal.roll == roll and
                    self.attitude_servo_goal.pitch == pitch and
                    self.attitude_servo_goal.yaw == yaw and
                    self.attitude_servo_goal.speed_knots == speed_knots and
                    self.attitude_servo_goal.ena_mask == 15)
        self.assertTrue(wait_for(attitude_servo_goals_are_set),
                        msg='Mission control must publish goals')

        rospy.sleep(2)
        # send data to finish the mission
        rpy_data = Vector3()
        pose_estimator_corrected_data = CorrectedData()
        rpy_data.x = 1.0
        rpy_data.y = 1.0
        rpy_data.z = 1.0
        pose_estimator_corrected_data.rpy_ang = rpy_data
        self.simulated_pose_estimator_pub.publish(
            pose_estimator_corrected_data)

        def success_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported),
                        msg='Mission control must report SUCCESS')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'test_mission_with_attitude_servo_behavioral',
                   TestAttitudeServoBehavior)
