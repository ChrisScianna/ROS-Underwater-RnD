import os
import collections
import datetime
import subprocess
import signal

import rospy

from .common import textindent
from .mission_logger import MissionLogger


class MissionBagRecorder(MissionLogger):

    Recording = collections.namedtuple('Recording', ['subprocess'])

    def __init__(self, name='mission_bag_recorder', anonymous=True):
        super(MissionBagRecorder, self).__init__(name, anonymous=anonymous)

    def _start(self, mission_id):
        now = datetime.datetime.now()
        bag_name = 'mission_{}_{}.bag'.format(
            mission_id, now.strftime('%Y%m%d_%H%M%S')
        )
        cmd = ['rosbag', 'record', '-O', bag_name]
        if self._topics:
            cmd.extend(self._topics)
        else:
            cmd.append('--all')
        rospy.loginfo(
            "Bagfile '%s' for mission %d to be written to '%s'",
            bag_name, mission_id, self._log_directory
        )
        return MissionBagRecorder.Recording(
            subprocess.Popen(
                cmd, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self._log_directory,
                preexec_fn=os.setsid,
            )
        )

    def _check(self, recording):
        return recording.subprocess.poll() is None

    def _stop(self, recording):
        p = recording.subprocess
        if p.poll() is None:
            pgrp = os.getpgid(p.pid)
            os.killpg(pgrp, signal.SIGINT)
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            rospy.logerr('rosbag record finished with nonzero exit code %d', p.returncode)
            rospy.logdebug('rosbag captured stdout')
            rospy.logdebug(textindent(stdout))
            rospy.logdebug('rosbag captured stderr')
            rospy.logdebug(textindent(stderr))
