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
import unittest

import rospy
import rostest

from mission_control.msg import FixedRudder

from test_utilities import MissionControlInterface


class TestFixRudderAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('fix_rudder_test_node')

    def setUp(self):
        self.mission_control = MissionControlInterface()

    def test_mission_that_fixes_rudder(self):
        """
        This test:
        -   Loads a mission with a single FixRudder action.
        -   Executes the mission.
        -   Waits until the mission is COMPLETE.
        """
        depth = 1.0
        rudder = 2.0
        speed_knots = 3.0
        ena_mask = FixedRudder.DEPTH_ENA
        ena_mask |= FixedRudder.RUDDER_ENA
        ena_mask |= FixedRudder.SPEED_KNOTS_ENA

        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <Sequence name="Test Mission">
                <FixRudder
                    depth="{}"
                    rudder="{}"
                    speed_knots="{}"/>
              </Sequence>
            </BehaviorTree>
          </root>
        '''.format(depth, rudder, speed_knots)

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)

        msg = rospy.wait_for_message(
            '/mngr/fixed_rudder', FixedRudder)
        self.assertEqual(msg.depth, depth)
        self.assertEqual(msg.rudder, rudder)
        self.assertEqual(msg.speed_knots, speed_knots)
        self.assertEqual(msg.ena_mask, ena_mask)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))


if __name__ == '__main__':
    rostest.rosrun(
        'mission_control',
        'test_fix_rudder_action',
        TestFixRudderAction)
