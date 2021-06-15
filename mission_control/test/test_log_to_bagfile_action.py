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
import os
import tempfile
import textwrap
import unittest

import rosbag
import rospy
import rostest

from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import ReportLoadMissionState
from mission_interface import MissionInterface
from mission_interface import wait_for


class TestLogToBagfileAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('node_to_test_log_to_bagfile')

    def setUp(self):
        self.mission_interface = MissionInterface()

    def _log_one_topic_to_bagfile(self, compression_type):
        bag_name = 'one_topic.{}.bag'.format(compression_type)
        topic_name = '/mngr/fixed_rudder'
        with tempfile.NamedTemporaryFile() as f:
            f.write(textwrap.dedent('''
            <root main_tree_to_execute="main">
              <BehaviorTree ID="main">
                <LogToBagfile prefix="{}" topics="{}" compression="{}">
                  <Sequence>
                    <FixRudder depth="1.0" rudder="2.0" speed_knots="3.0"/>
                    <Delay delay_msec="1000">
                      <AlwaysSuccess/>
                    </Delay>
                  </Sequence>
                </LogToBagfile>
              </BehaviorTree>
            </root>'''.format(bag_name, topic_name, compression_type)))
            f.flush()

            result = self.mission_interface.load_mission(f.name)
            self.assertEqual(result.load_state, ReportLoadMissionState.SUCCESS)
            mission_id = result.mission_id

        self.mission_interface.execute_mission(mission_id)

        def mission_completed():
            return ReportExecuteMissionState.COMPLETE in \
                self.mission_interface.execute_mission_state
        self.assertTrue(wait_for(mission_completed),
                        msg='Mission did not COMPLETE')

        bag_path = os.path.join(
            os.path.dirname(rospy.core._log_filename), bag_name)
        self.assertTrue(os.path.isfile(bag_path), bag_path + ' is missing')
        with rosbag.Bag(bag_path, 'r') as bag:
            self.assertEqual(bag.get_compression_info().compression, compression_type)
            topics = bag.get_type_and_topic_info().topics
            self.assertEqual(len(topics), 1)
            name, info = topics.items()[0]
            self.assertEqual(name, topic_name)
            self.assertEqual(info.message_count, bag.get_message_count())

    def test_log_to_bagfile(self):
        self._log_one_topic_to_bagfile(compression_type='none')

    def test_log_to_bz2_compressed_bagfile(self):
        self._log_one_topic_to_bagfile(compression_type='bz2')

    def test_log_to_lz4_compressed_bagfile(self):
        self._log_one_topic_to_bagfile(compression_type='lz4')

    def test_log_all_to_bagfile(self):
        bag_name = 'all_topics.bag'
        with tempfile.NamedTemporaryFile() as f:
            f.write(textwrap.dedent('''
            <root main_tree_to_execute="main">
              <BehaviorTree ID="main">
                <LogToBagfile prefix="{}">
                  <Delay delay_msec="1000">
                    <AlwaysSuccess/>
                  </Delay>
                </LogToBagfile>
              </BehaviorTree>
            </root>'''.format(bag_name)))
            f.flush()

            result = self.mission_interface.load_mission(f.name)
            self.assertEqual(result.load_state, ReportLoadMissionState.SUCCESS)
            mission_id = result.mission_id

        self.mission_interface.execute_mission(mission_id)

        def mission_completed():
            return ReportExecuteMissionState.COMPLETE in \
                self.mission_interface.execute_mission_state
        self.assertTrue(wait_for(mission_completed),
                        msg='Mission did not COMPLETE')

        bag_path = os.path.join(
            os.path.dirname(rospy.core._log_filename), bag_name)
        self.assertTrue(os.path.isfile(bag_path), bag_path + ' is missing')
        with rosbag.Bag(bag_path, 'r') as bag:
            topics = bag.get_type_and_topic_info().topics
            self.assertGreaterEqual(len(topics), 1)
            self.assertIn('/rosout', topics)


if __name__ == '__main__':
    rostest.rosrun('mission_control', 'test_log_to_bagfile_action', TestLogToBagfileAction)
