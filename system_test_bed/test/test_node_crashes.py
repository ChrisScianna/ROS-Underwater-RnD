#!/usr/bin/env python
'''
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
'''

import unittest
import rostest
import rospy
import os
import sys
from std_msgs.msg import String
from health_monitor.msg import ReportFault


class TestNodeCrashes(unittest.TestCase):

    fault_error = 0
    fault_count = 0
    fault_values = {
        "payload_manager": ReportFault.PAYLOAD_NODE_DIED,
        "vectornav": ReportFault.AHRS_NODE_DIED,
        "pressure_sensor": ReportFault.PRESSURE_NODE_DIED,
        "mission_manager": ReportFault.MISSION_NODE_DIED,
        "pose_estimator": ReportFault.POSE_NODE_DIED,
        "thruster_control_node": ReportFault.THRUSTER_NODE_DIED,
        "autopilot": ReportFault.AUTOPILOT_NODE_DIED,
        "battery_monitor": ReportFault.BATTERY_NODE_DIED,
        "jaus_node_bridge": ReportFault.JAUS_NODE_DIED
    }

    def callback(self, msg):
        self.fault_error |= msg.fault_id
        self.fault_count += 1

    def test_node_crash(self):
        node_name = sys.argv[1]
        error_code = self.fault_values[node_name]
        rospy.init_node('rosmon_status')
        rospy.Subscriber('/health_monitor/report_fault',
                         ReportFault, self.callback)
        cmd = "kill -9 $(rosnode info " + node_name + \
            " 2>/dev/null | grep Pid| cut -d' ' -f2)"
        os.system(cmd)
        while self.fault_count < 2:
            pass

        self.assertTrue(self.fault_error, error_code)


if __name__ == "__main__":
    rostest.rosrun('system_test_bed', 'node_crash', TestNodeCrashes)
