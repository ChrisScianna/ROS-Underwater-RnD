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

from test_utilities import MissionControlInterface
from test_utilities import wait_for


class TestMissionControlInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('mission_control_interface_test_node')

    def setUp(self):
        self.mission_control = MissionControlInterface()

    def test_bad_mission_loads(self):
        self.assertIsNone(
            self.mission_control.load_mission_from_file('not_a_mission_file'))

        self.assertIsNone(
            self.mission_control.load_mission('not_a_mission_definition'))

    def test_mission_loading_and_removal(self):
        mission_description = 'test mission'
        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <AlwaysSuccess name="{}"/>
            </BehaviorTree>
          </root>
        '''.format(mission_description)

        self.mission_control.remove_missions()

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        missions = self.mission_control.query_missions()
        self.assertEqual(len(missions), 1)
        mission = missions[0]
        self.assertEqual(mission.mission_id, mission_id)
        self.assertEqual(mission.mission_description, mission_description)

        self.mission_control.remove_missions()

        def all_missions_removed():
            return len(self.mission_control.query_missions()) == 0
        self.assertTrue(wait_for(all_missions_removed))

    def test_mission_execution(self):
        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <Delay delay_msec="1000">
                <AlwaysSuccess/>
              </Delay>
            </BehaviorTree>
          </root>
        '''

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))

    def test_mission_restart_after_stop_during_abort(self):
        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <Delay delay_msec="2000">
                <AlwaysSuccess/>
              </Delay>
            </BehaviorTree>
          </root>
        '''

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)
        # NOTE(hidmic): this is a fundamental flaw in the interface
        # There's no guarantee w.r.t message arrival order
        rospy.sleep(0.5)
        self.mission_control.abort_mission(mission_id)
        self.mission_control.stop_mission()

        self.assertTrue(self.mission_control.wait_for_abort(mission_id))

        self.mission_control.execute_mission(mission_id)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))

    def test_regular_heartbeats(self):
        def enough_heartbeats():
            return len(self.mission_control.heartbeats) >= 5
        self.assertTrue(wait_for(enough_heartbeats))
        timestamps = [
            heartbeat.header.stamp
            for heartbeat in
            self.mission_control.heartbeats]
        average_period = sum(
            (t1 - t0).to_sec() for t0, t1 in zip(
                timestamps[:-1], timestamps[1:])
        ) / (len(timestamps) - 1)
        expected_period = 1.0 / self.mission_control.heartbeat_rate
        self.assertAlmostEqual(average_period, expected_period, places=3)


if __name__ == '__main__':
    rostest.rosrun(
        'mission_control',
        'test_mission_control_interface',
        TestMissionControlInterface)
