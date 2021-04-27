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

from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import Waypoint

from mission_interface import MissionInterface
from mission_interface import wait_for


class TestGoToWaypointAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_go_to_waypoint')

    def setUp(self):
        self.waypoint = None
        self._waypoint_sub = rospy.Subscriber(
            '/mngr/waypoint', Waypoint,
            self._waypoint_callback)
        self._state_pub = rospy.Publisher(
            '/state', StateStamped, queue_size=1)
        self.mission = MissionInterface()

    def _waypoint_callback(self, msg):
        self.waypoint = msg

    def test_mission_that_goes_to_waypoint(self):
        self.assertTrue(self.mission.load_mission('waypoint_mission_test.xml'))

        self.mission.execute_mission()

        def waypoint_is_set():
            return self.waypoint is not None
        self.assertTrue(wait_for(waypoint_is_set), msg='No waypoint was set')

        ena_mask = Waypoint.LAT_ENA | Waypoint.LONG_ENA
        ena_mask |= Waypoint.ALTITUDE_ENA
        ena_mask |= Waypoint.SPEED_KNOTS_ENA
        self.assertEqual(self.waypoint.ena_mask, ena_mask)

        self.mission.read_behavior_parameters('GoToWaypoint')
        latitude = float(self.mission.get_behavior_parameter('latitude'))
        longitude = float(self.mission.get_behavior_parameter('longitude'))
        altitude = float(self.mission.get_behavior_parameter('altitude'))
        speed_knots = float(self.mission.get_behavior_parameter('speed_knots'))

        self.assertEqual(self.waypoint.latitude, latitude)
        self.assertEqual(self.waypoint.longitude, longitude)
        self.assertEqual(self.waypoint.altitude, altitude)
        self.assertEqual(self.waypoint.speed_knots, speed_knots)

        msg = StateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.state.geolocation.position.latitude = latitude
        msg.state.geolocation.position.longitude = longitude
        msg.state.geolocation.position.altitude = altitude
        self._state_pub.publish(msg)

        def mission_complete():
            return (ReportExecuteMissionState.ABORTING not in self.mission.execute_mission_state and
                    ReportExecuteMissionState.COMPLETE == self.mission.execute_mission_state[-1])
        self.assertTrue(wait_for(mission_complete), msg='Mission did not complete')


if __name__ == '__main__':
    rostest.rosrun('mission_control', 'test_go_to_waypoint_action', TestGoToWaypointAction)
