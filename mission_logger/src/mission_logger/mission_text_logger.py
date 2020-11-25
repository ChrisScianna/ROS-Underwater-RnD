import os
import collections
import datetime
import signal
import subprocess

import rosgraph
import roslib.message
from rostopic import _str_plot as message_to_csv
from rostopic import _str_plot_fields as message_fields_to_csv

import rospy

from .mission_logger import MissionLogger


class MissionTextLogger(MissionLogger):

    Recording = collections.namedtuple(
        'Recording', ['subscribers', 'files', 'errors']
    )

    def __init__(self, name='mission_text_logger', anonymous=True):
        super(MissionTextLogger, self).__init__(name, anonymous=anonymous)

    def _start(self, mission_id):
        master = rosgraph.Master(rospy.get_name())
        topics = master.getPublishedTopics('')
        if self._topics:
            topics = [
                (topic_name, topic_type)
                for topic_name, topic_type in topics
                if topic_name in self._topics
            ]
        now = datetime.datetime.now()
        output_directory = os.path.join(
            self._log_directory, 'mission_{}_{}'.format(
                mission_id, now.strftime('%Y%m%d_%H%M%S')
            )
        )
        os.makedirs(output_directory)
        rospy.loginfo(
            "Text logs for mission %d to be written to '%s'",
            mission_id, output_directory
        )
        recording = MissionTextLogger.Recording(
            subscribers=[], files=[], errors=[]
        )
        try:
            for topic_name, topic_type in topics:
                msg_class = roslib.message.get_message_class(topic_type)
                output_filename = topic_name.strip('/').replace('/', '_') + '.txt'
                output_filepath = os.path.join(output_directory, output_filename)
                output_file = open(output_filepath, 'w')
                recording.files.append(output_file)
                output_file.write('%' + message_fields_to_csv(
                    msg_class(), 'field', None
                ) + '\n')
                recording.subscribers.append(rospy.Subscriber(
                    topic_name, msg_class, self._write_message, (recording, output_file)
                ))
        except Exception:
            self._stop(recording)
            raise
        return recording

    def _write_message(self, data, args):
        recording, output_file = args
        try:
            if not os.path.exists(output_file.name):
                raise RuntimeError('{} does not exist'.format(output_file.name))
            output_file.write(message_to_csv(data) + '\n')
        except Exception as e:
            recording.errors.append(e)

    def _check(self, recording):
        return not recording.errors

    def _stop(self, recording):
        for sub in recording.subscribers:
            sub.unregister()
        for f in recording.files:
            f.close()
        if recording.errors:
            rospy.logerr('%d errors while writing messages', len(recording.errors))
            for error in recording.errors:
                rospy.logdebug(error)
