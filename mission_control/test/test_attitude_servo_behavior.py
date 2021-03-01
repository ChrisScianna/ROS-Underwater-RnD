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
from mission_control.msg import AttitudeServo
from pose_estimator.msg import CorrectedData
from geometry_msgs.msg import Vector3


def wait_for(predicate, period=1):
    while not rospy.is_shutdown():
        result = predicate()
        if result:
            return result
        rospy.sleep(period)
    return predicate()


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
        mission_execute_state = ReportExecuteMissionState.PAUSED
        mission_load_state = False
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) + '/test_files/'
        mission_load_state_flag = False
        report_mission_flag = False
        report_mission = ReportMissions()
        self.mission_load_state = None
        self.report_mission = None
        self.report_execute_mission = None
        self.attitude_servo_goal = []

        # Subscribers
        self.exec_state_sub = rospy.Subscriber('/mission_control_node/report_mission_execute_state',
                                               ReportExecuteMissionState, self.callback_mission_execute_state)

        self.load_state_sub = rospy.Subscriber('/mission_control_node/report_mission_load_state',
                                               ReportLoadMissionState, self.callback_mission_load_state)

        self.report_missions_sub = rospy.Subscriber('/mission_control_node/report_missions',
                                                    ReportMissions, self.callback_report_mission)

        self.attitude_servo_msg = rospy.Subscriber('/mngr/attitude_servo',
                                                 AttitudeServo, self.callback_publish_attitude_servo_goal)

        # Publishers
        self.simulated_mission_control_load_mission_pub = rospy.Publisher('/mission_control_node/load_mission',
                                                                          LoadMission, latch=True, queue_size=1)

        self.simulated_mission_control_execute_mission_pub = rospy.Publisher(
            '/mission_control_node/execute_mission', ExecuteMission, latch=True, queue_size=1)

        self.simulated_pose_estimator_pub = rospy.Publisher('/pose/corrected_data',
                                                       CorrectedData, queue_size=1)

    def callback_mission_execute_state(self, msg):
        self.report_execute_mission = msg

    def callback_report_mission(self, msg):
        self.report_mission = msg

    def callback_mission_load_state(self, msg):
        self.mission_load_state = msg.load_state

    def callback_publish_attitude_servo_goal(self, msg):
        self.attitude_servo_goal = [msg.roll,
                                    msg.pitch,
                                    msg.yaw,
                                    msg.speed_knots,
                                    msg.ena_mask
                                    ]
        rospy.loginfo(self.attitude_servo_goal)

    def test_mission_with_attitude_servo_behavioral(self):

        self.mission_load_state_flag = False
        mission_to_load = LoadMission()
        mission_to_load.mission_file_full_path = self.dir_path + "test_missions/attitude_servo_mission_test.xml"
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

        def attitude_servo_goals_are_setted():
            return self.attitude_servo_goal == [1.0, 1.0, 1.0, 3.0, 15]
        self.assertTrue(wait_for(attitude_servo_goals_are_setted),
                        msg='Mission control must publish goals')
        
        #send data to finish the mission
        rpy_data = Vector3()
        pose_estimator_corrected_data = CorrectedData()
        rpy_data.x = 1.0
        rpy_data.y = 1.0
        rpy_data.z = 1.0
        pose_estimator_corrected_data.rpy_ang = rpy_data
        self.simulated_pose_estimator_pub.publish(
            pose_estimator_corrected_data)

        def success_mission_status_is_reported():
            return self.report_execute_mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(success_mission_status_is_reported), 
            msg='Mission control must report SUCCESS')
        

if __name__ == "__main__":
    rostest.rosrun('mission_control', 'test_mission_with_attitude_servo_behavioral',
                   TestAttitudeServoBehavior)
