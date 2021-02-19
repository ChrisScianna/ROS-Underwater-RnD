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
import rosnode
import rospy
import rostest
from mission_control.msg import ReportHeartbeat


class TestMissionControlPublishesHeartbeat(unittest.TestCase):
    """
        this test checks if mission control publishes heartbeats.
    """

    @classmethod
    def setUpClass(cls):
        rospy.init_node('heartbeat_mission_control')

    def setUp(self):
        self.mission_manager_heart_beat_count = 0

    def callback_mission_manager_heart_beat(self, msg):
        self.mission_manager_heart_beat_count = msg.seq_id

    def test_mission_control_published_heartbeat(self):
        rospy.Subscriber('/mission_control_node/report_heartbeat',
                         ReportHeartbeat, self.callback_mission_manager_heart_beat)

        while not rospy.is_shutdown() and self.mission_manager_heart_beat_count < 3:
            rospy.sleep(0.1)

        self.assertTrue(self.mission_manager_heart_beat_count >= 3)


if __name__ == "__main__":
    rostest.rosrun('mission_control_node', 'mission_control_heartbeat',
                   TestMissionControlPublishesHeartbeat)