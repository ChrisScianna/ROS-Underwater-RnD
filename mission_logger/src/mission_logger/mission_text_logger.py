import os
import collections
import datetime
import signal
import subprocess

import rosgraph
import rospy

from .common import textindent
from .mission_logger import MissionLogger


class MissionTextLogger(MissionLogger):

    Recording = collections.namedtuple(
        'Recording', ['subprocesses', 'files']
    )

    def __init__(self, name='mission_text_logger', anonymous=True):
        super(MissionTextLogger, self).__init__(name, anonymous=anonymous)

    def _start(self, mission_id):
        if not self._topics:
            master = rosgraph.Master(rospy.get_name())
            pubs, _, _ = master.getSystemState()
            topics = [topic for topic, _ in pubs]
        else:
            topics = self._topics
        now = datetime.datetime.now()
        output_directory = os.path.join(
            self._log_directory, 'mission_{}_{}'.format(
                mission_id, now.strftime('%Y%m%d_%H%M%S')
            )
        )
        os.makedirs(output_directory)
        recording = MissionTextLogger.Recording(subprocesses=[], files=[])
        for topic in topics:
            output_filename = topic.strip('/').replace('/', '_') + '.txt'
            output_filepath = os.path.join(output_directory, output_filename)
            output_file = open(output_filepath, 'w')
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'  # avoid output buffering
            recording.subprocesses.append(subprocess.Popen(
                ['rostopic', 'echo', '-p', topic], env=env,
                stdout=output_file, stderr=subprocess.PIPE,
            ))
            recording.files.append(output_file)
        return recording

    def _check(self, recording):
        return all(p.poll() is None for p in recording.subprocesses)

    def _stop(self, recording):
        for p in recording.subprocesses:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p, f in zip(recording.subprocesses, recording.files):
            _, stderr = p.communicate()
            if p.returncode != 0:
                rospy.logerr('rostopic record finished with nonzero exit code %d', p.returncode)
                rospy.logdebug('rostopic captured stderr')
                rospy.logdebug(textindent(stderr))
            f.close()
