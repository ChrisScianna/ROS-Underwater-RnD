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
from health_monitor.msg import ReportFault
from mission_control.msg import AttitudeServo
from thruster_control.msg import ReportRPM
from auv_interfaces.msg import StateStamped
from mission_interface import MissionInterface
from mission_interface import wait_for


def isclose(a, b, tol):
    return abs(a - b) < tol


class TestMissionControlAbortsWhenHealthMonitorReportsFault(unittest.TestCase):
    """
        Simulate error message sent by health monitor and checks
        if the mission control publishes the abort status.
        The steps int the involved are:
            1)  Load a mission
            2)  Execute the Mission
            3)  Simulate an error sent by health monitor
            4)  check if the mission control aborts the mission
                a) Receive Aborting State
                b) Set the fins to surface and Thruster velocity to 0 RPM
                c) The abort behavior is complete
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_mission_control_aborts_procedure')

    def setUp(self):
        self.attitude_servo_goal = AttitudeServo()
        self.mission = MissionInterface()
        self.simulate_error_code = ReportFault()
        self.simulate_error_code.fault_id = ReportFault.AUTOPILOT_NODE_DIED
        self.attitude_servo_aborting_goal = AttitudeServo()

        self.attitude_servo_msg = rospy.Subscriber(
            '/mngr/attitude_servo',
            AttitudeServo,
            self.attitude_servo_callback)

        self.simulated_health_monitor_pub = rospy.Publisher(
            '/health_monitor/report_fault',
            ReportFault, queue_size=1)

        self.simulated_auv_interface_data_pub = rospy.Publisher(
            '/state',
            StateStamped,
            queue_size=1)

        self.simulated_thruster_velocity_pub = rospy.Publisher(
            '/thruster_control/report_rpm',
            ReportRPM, queue_size=1)

    def attitude_servo_callback(self, msg):
        self.attitude_servo_aborting_goal = msg

    def test_mission_control_aborts_if_health_monitor_reports_fault(self):
        self.mission.load_mission('attitude_servo_mission_test.xml')
        self.mission.execute_mission()

        def executing_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.EXECUTING
        self.assertTrue(wait_for(executing_mission_status_is_reported),
                        msg='Mission control must report EXECUTING')

        # Simulate the health monitor publishing the fault code
        self.simulated_health_monitor_pub.publish(self.simulate_error_code)

        def aborting_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.ABORTING
        self.assertTrue(wait_for(aborting_mission_status_is_reported),
                        msg='Mission control must report aborting')

        maxCtrlFinAngle = rospy.get_param('/fin_control/max_ctrl_fin_angle')
        def attitude_servo_aborting_goals_are_set():
            tol = 1e-3
            return (isclose(self.attitude_servo_aborting_goal.roll, 0.0, tol) and
                    isclose(self.attitude_servo_aborting_goal.pitch, maxCtrlFinAngle, tol) and
                    isclose(self.attitude_servo_aborting_goal.yaw, 0.0, tol) and
                    isclose(self.attitude_servo_aborting_goal.speed_knots, 0.0, tol) and
                    self.attitude_servo_aborting_goal.ena_mask == 0xF)
        self.assertTrue(wait_for(attitude_servo_aborting_goals_are_set),
                        msg='Mission control must publish goals')

        # Publishes values sent to the actuators
        msg = StateStamped()
        msg.state.manoeuvring.pose.mean.orientation.x = 0.0
        msg.state.manoeuvring.pose.mean.orientation.y = maxCtrlFinAngle
        msg.state.manoeuvring.pose.mean.orientation.z = 0.0
        self.simulated_auv_interface_data_pub.publish(msg)

        thruster_velocity = ReportRPM()
        thruster_velocity.rpms = 0
        self.simulated_thruster_velocity_pub.publish(thruster_velocity)

        def complete_mission_status_is_reported():
            return self.mission.execute_mission_state == ReportExecuteMissionState.COMPLETE
        self.assertTrue(wait_for(complete_mission_status_is_reported),
                        msg='Mission control must report COMPLETE')


if __name__ == "__main__":
    rostest.rosrun('mission_control', 'mission_control_aborts_when_health_monitor_reports_fault',
                   TestMissionControlAbortsWhenHealthMonitorReportsFault)
