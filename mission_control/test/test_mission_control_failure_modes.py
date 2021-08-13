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
import unittest

import rospy
import rostest

from auv_interfaces.msg import StateStamped
from health_monitor.msg import ReportFault
from mission_control.msg import AttitudeServo

from test_utilities import MissionControlInterface


class TestMissionControlFailureModes(unittest.TestCase):
    """
        Test if the mission control aborts after the behavior returns Failure state.
        The failure is produced because the mission time is greater than the time_out
        mission behavior value

        The steps involved are:
            1)  Load a mission
            2)  Execute the mission
            4)  Wait the mission control to publish a Failure state
            5)  check if the mission control aborts the mission
                a) Receive Aborting State
                b) Set the fins to surface and Thruster velocity to 0 RPM
                c) The abort behavior is complete
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('mission_control_failure_modes_test_node')

    def setUp(self):
        self.mission_control = MissionControlInterface()

    def test_mission_control_aborts_on_request(self):
        def abort_request(mission_id):
            self.mission_control.abort_mission(mission_id)
        self._check_mission_control_aborts_on(abort_request)

    def test_mission_control_aborts_on_fault(self):
        def fault_report(*args):
            msg = ReportFault()
            msg.fault_id = ReportFault.AUTOPILOT_NODE_DIED
            rospy.Publisher(
                '/health_monitor/report_fault',
                ReportFault, queue_size=1
            ).publish(msg)
        self._check_mission_control_aborts_on(fault_report)

    def test_mission_control_aborts_on_timeout(self):
        def mission_timeout(*args):
            rospy.sleep(1.0)
        self._check_mission_control_aborts_on(mission_timeout)

    def _check_mission_control_aborts_on(self, trigger_mission_event):
        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <TimeoutAfter msec="1000">
                <DelayFor delay_msec="10000">
                  <AlwaysSuccess/>
                </DelayFor>
              </TimeoutAfter>
            </BehaviorTree>
          </root>
        '''

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)
        rospy.sleep(0.5)
        trigger_mission_event(mission_id)

        max_ctrl_fin_angle = rospy.get_param('/fin_control/max_ctrl_fin_angle')
        msg = rospy.wait_for_message('/mngr/attitude_servo', AttitudeServo)
        ena_mask = AttitudeServo.PITCH_ENA | AttitudeServo.SPEED_KNOTS_ENA
        self.assertEqual(msg.ena_mask, ena_mask)
        self.assertAlmostEqual(msg.pitch, -max_ctrl_fin_angle)
        self.assertEqual(msg.speed_knots, 0.0)

        msg = StateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.state.manoeuvring.pose.mean.orientation.y = -max_ctrl_fin_angle
        msg.state.manoeuvring.velocity.mean.linear.x = 0.0
        msg.state.manoeuvring.velocity.mean.linear.y = 0.0
        msg.state.manoeuvring.velocity.mean.linear.z = 0.0
        rospy.Publisher(
            '/state', StateStamped,
            queue_size=1, latch=True
        ).publish(msg)

        self.assertTrue(self.mission_control.wait_for_abort(mission_id))


if __name__ == '__main__':
    rostest.rosrun(
        'mission_control',
        'test_mission_control_failure_modes',
        TestMissionControlFailureModes)
