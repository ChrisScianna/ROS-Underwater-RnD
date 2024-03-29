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
import glob
import os
import unittest

import rosbag
import rospy
import rostest

from test_utilities import MissionControlInterface


class TestLogToBagfileAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('log_to_bagfile_test_node')

    def setUp(self):
        self.mission_control = MissionControlInterface()

    def _log_some_topics_to_bagfile(self, compression_type):
        bag_name = 'some_topics.{}.bag'.format(compression_type)
        topic_names = ['/mngr/fixed_rudder', '/rosout']

        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <LogToBagfile prefix="{}" topics="{}" compression="{}">
                <Sequence>
                  <FixRudder depth="1.0" rudder="2.0" speed_knots="3.0"/>
                  <DelayFor delay_msec="1000">
                    <AlwaysSuccess/>
                  </DelayFor>
                </Sequence>
              </LogToBagfile>
            </BehaviorTree>
          </root>
        '''.format(bag_name, ', '.join(topic_names), compression_type)

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))

        bag_path = os.path.join(
            os.path.dirname(rospy.core._log_filename), bag_name)
        self.assertTrue(os.path.isfile(bag_path), bag_path + ' is missing')
        with rosbag.Bag(bag_path, 'r') as bag:
            compression_info = bag.get_compression_info()
            self.assertEqual(compression_type, compression_info.compression)
            topics = bag.get_type_and_topic_info().topics
            self.assertEqual(len(topics), len(topic_names))
            for topic_name in topic_names:
                self.assertIn(topic_name, topics)

    def test_log_to_bagfile(self):
        self._log_some_topics_to_bagfile(compression_type='none')

    def test_log_to_bz2_compressed_bagfile(self):
        self._log_some_topics_to_bagfile(compression_type='bz2')

    def test_log_to_lz4_compressed_bagfile(self):
        self._log_some_topics_to_bagfile(compression_type='lz4')

    def test_log_all_to_bagfile(self):
        bag_prefix = 'all_topics'

        mission_definition = '''
          <root main_tree_to_execute="main">
            <BehaviorTree ID="main">
              <LogToBagfile prefix="{}">
                <DelayFor delay_msec="1000">
                  <AlwaysSuccess/>
                </DelayFor>
              </LogToBagfile>
            </BehaviorTree>
          </root>
        '''.format(bag_prefix)

        mission_id = self.mission_control.load_mission(mission_definition)
        self.assertIsNotNone(mission_id)

        self.mission_control.execute_mission(mission_id)

        self.assertTrue(self.mission_control.wait_for_completion(mission_id))

        full_bag_prefix = os.path.join(
            os.path.dirname(rospy.core._log_filename), bag_prefix)
        candidates = glob.glob(full_bag_prefix + '*.bag')
        self.assertEqual(len(candidates), 1)
        bag_path = candidates[0]
        self.assertTrue(os.path.isfile(bag_path))
        with rosbag.Bag(bag_path, 'r') as bag:
            topics = bag.get_type_and_topic_info().topics
            self.assertGreaterEqual(len(topics), 1)
            self.assertIn('/rosout', topics)


if __name__ == '__main__':
    rostest.rosrun(
        'mission_control',
        'test_log_to_bagfile_action',
        TestLogToBagfileAction)
