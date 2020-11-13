import threading

import rospkg
import rospy

from mission_manager.msg import ReportExecuteMissionState


class MissionLogger(object):

    ACTIVE_MISSION_STATES = (
        ReportExecuteMissionState.ABORTING,
        ReportExecuteMissionState.EXECUTING,
        ReportExecuteMissionState.PAUSED
    )

    def __init__(self, *args, **kwargs):
        rospy.init_node(*args, **kwargs)
        self._topics = rospy.get_param('~topics', [])
        self._check_period = rospy.get_param('~check_period', 0.1)
        self._log_directory = rospy.get_param(
            '~log_directory', rospkg.get_log_dir()
        )
        self._lock = threading.Lock()
        self._mission_recordings = {}

    def _start(self, mission_id):
        raise NotImplementedError()

    def _check(self, recording):
        raise NotImplementedError()

    def _stop(self, recording):
        raise NotImplementedError()

    def _on_mission_state_change(self, msg):
        with self._lock:
            if msg.execute_mission_state in MissionLogger.ACTIVE_MISSION_STATES:
                if msg.mission_id not in self._mission_recordings:
                    rospy.loginfo('Mission %d is active, start recording', msg.mission_id)
                    self._mission_recordings[msg.mission_id] = self._start(msg.mission_id)
                    rospy.loginfo('Mission %d recording started', msg.mission_id)
            else:
                if msg.mission_id in self._mission_recordings:
                    rospy.loginfo('Mission %d done, stop recording', msg.mission_id)
                    self._stop(self._mission_recordings[msg.mission_id])
                    del self._mission_recordings[msg.mission_id]
                    rospy.loginfo('Mission %d recording stopped', msg.mission_id)

    def spin(self):
        mission_state_subscriber = rospy.Subscriber(
            "/mngr/report_mission_execute_state",
            ReportExecuteMissionState, self._on_mission_state_change
        )
        try:
            while not rospy.is_shutdown():
                with self._lock:
                    for mission_id in list(self._mission_recordings.keys()):
                        ok = self._check(self._mission_recordings[mission_id])
                        if not ok:
                            rospy.logerr('Had issues with mission %d recording', mission_id)
                            self._stop(self._mission_recordings[mission_id])
                            del self._mission_recordings[mission_id]
                            rospy.loginfo('Mission %d recording stopped', mission_id)
                rospy.sleep(self._check_period)
        finally:
            with self._lock:
                if self._mission_recordings:
                    rospy.loginfo('Stopping all active mission recordings')
                    for mission_id, recording in self._mission_recordings.items():
                        self._stop(recording)
                        rospy.loginfo('Mission %d recording stopped', mission_id)
                    self._mission_recordings.clear()
            mission_state_subscriber.unregister()
