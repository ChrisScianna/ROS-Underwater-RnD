import os
import collections
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
        bag_prefix = 'mission_{}'.format(mission_id)
        cmd = ['rosbag', 'record', '-o', bag_prefix]
        if self._topics:
            cmd.extend(self._topics)
        else:
            cmd.append('--all')
        return MissionBagRecorder.Recording(
            subprocess.Popen(
                cmd, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=self._log_directory
            )
        )

    def _check(self, recording):
        return recording.subprocess.poll() is None

    def _stop(self, recording):
        p = recording.subprocess
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            rospy.logerr('rosbag record finished with nonzero exit code %d', p.returncode)
            rospy.logdebug('rosbag captured stdout')
            rospy.logdebug(textindent(stdout))
            rospy.logdebug('rosbag captured stderr')
            rospy.logdebug(textindent(stderr))
