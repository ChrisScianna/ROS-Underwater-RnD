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
import os
import tempfile
import textwrap
import unittest

import rospy
import rostest

from mission_control.msg import AbortMission
from mission_control.msg import ExecuteMission
from mission_control.msg import LoadMission
from mission_control.msg import QueryMissions
from mission_control.msg import RemoveMissions
from mission_control.msg import ReportExecuteMissionState
from mission_control.msg import ReportHeartbeat
from mission_control.msg import ReportLoadMissionState
from mission_control.msg import ReportMissions

from std_msgs.msg import Empty

from .waits import wait_for


class MissionControlInterface:
    """
        This class:
        - simulates the Jaus Ros Bridge commands:
            a) Load Mission
            b) Execute Mission
            c) Subscribe to:
                -   report_mission_execute_state
                -   report_mission_load_state
        - Helper functions:
            a) Read mission parameters
    """

    def __init__(self, ns='/mngr'):
        self._reported_execution_states = {}
        self._exec_state_sub = rospy.Subscriber(
            ns + '/report_mission_execute_state',
            ReportExecuteMissionState,
            self._on_mission_execute_state)

        self._heartbeats = []
        self._hearbeats_sub = rospy.Subscriber(
            ns + '/report_heartbeat', ReportHeartbeat,
            self._on_heartbeat
        )
        self._heartbeat_rate = rospy.get_param(
            ns + '/heartbeat_rate')

        self._last_reported_load_state = None
        self._load_state_sub = rospy.Subscriber(
            ns + '/report_mission_load_state',
            ReportLoadMissionState,
            self._on_mission_load_state)

        self._load_mission_pub = rospy.Publisher(
            ns + '/load_mission', LoadMission,
            queue_size=1, latch=True)

        self._execute_mission_pub = rospy.Publisher(
            ns + '/execute_mission', ExecuteMission,
            queue_size=1, latch=True)

        self._stop_mission_pub = rospy.Publisher(
            ns + '/stop_missions', Empty,
            queue_size=1, latch=True)

        self._abort_mission_pub = rospy.Publisher(
            ns + '/abort_mission', AbortMission,
            queue_size=1, latch=True)

        self._reported_missions = None
        self._report_missions_sub = rospy.Subscriber(
            ns + '/report_missions', ReportMissions,
            self._on_missions_report)

        self._query_missions_pub = rospy.Publisher(
            ns + '/query_missions', QueryMissions,
            queue_size=1, latch=True)

        self._remove_missions_pub = rospy.Publisher(
            ns + '/remove_missions', RemoveMissions,
            queue_size=1, latch=True)

    def _on_missions_report(self, msg):
        self._reported_missions = msg.missions

    def _on_mission_execute_state(self, msg):
        states = self._reported_execution_states[msg.mission_id]
        if msg.execute_mission_state != states[-1]:
            states.append(msg.execute_mission_state)

    def _on_mission_load_state(self, msg):
        self._last_reported_load_state = msg

    def _on_heartbeat(self, msg):
        self._heartbeats.append(msg)

    @property
    def heartbeats(self):
        return list(self._heartbeats)

    @property
    def heartbeat_rate(self):
        return self._heartbeat_rate

    def load_mission_from_file(self, mission_path):
        msg = LoadMission()
        msg.header.stamp = rospy.Time.now()
        msg.mission_file_full_path = mission_path
        self._reported_load_state = None
        self._load_mission_pub.publish(msg)
        result = wait_for(lambda: self._last_reported_load_state)
        if result and result.load_state == ReportLoadMissionState.SUCCESS:
            return result.mission_id
        return None

    def load_mission(self, mission_definition):
        with tempfile.NamedTemporaryFile() as f:
            f.write(textwrap.dedent(mission_definition))
            f.flush()
            return self.load_mission_from_file(f.name)

    def execute_mission(self, mission_id):
        msg = ExecuteMission()
        msg.header.stamp = rospy.Time.now()
        msg.mission_id = mission_id
        self._execute_mission_pub.publish(msg)
        self._reported_execution_states[mission_id] = [
            ReportExecuteMissionState.EXECUTING]

    def abort_mission(self, mission_id):
        msg = AbortMission()
        msg.header.stamp = rospy.Time.now()
        msg.mission_id = mission_id
        self._abort_mission_pub.publish(msg)

    def wait_for_completion(self, mission_id):
        def completion_state_sequence_reported():
            states = self._reported_execution_states[mission_id]
            return states[-2:] == [
                ReportExecuteMissionState.EXECUTING,
                ReportExecuteMissionState.COMPLETE]
        return wait_for(completion_state_sequence_reported)

    def wait_for_abort(self, mission_id):
        def abort_state_sequence_reported():
            states = self._reported_execution_states[mission_id]
            return states[-3:] == [
                ReportExecuteMissionState.EXECUTING,
                ReportExecuteMissionState.ABORTING,
                ReportExecuteMissionState.COMPLETE]
        return wait_for(abort_state_sequence_reported)

    def query_missions(self):
        msg = QueryMissions()
        msg.header.stamp = rospy.Time.now()
        self._reported_missions = None
        self._query_missions_pub.publish(msg)
        if wait_for(lambda: self._reported_missions is not None):
            return self._reported_missions
        return []

    def remove_missions(self):
        msg = RemoveMissions()
        msg.header.stamp = rospy.Time.now()
        self._remove_missions_pub.publish(msg)

    def stop_mission(self):
        self._stop_mission_pub.publish(Empty())
