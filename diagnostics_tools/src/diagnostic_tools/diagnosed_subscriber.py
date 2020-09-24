import rospy


class DiagnosedSubscriber:

    def __init__(self, name, msg_class, callback, **kwargs):
        self._user_callback = callback
        self._sub = rospy.Subscriber(name, msg_class, self._callback, **kwargs)
        self._checks = []

    def _callback(self, message):
        now = rospy.Time.now()
        for task in self._checks:
            task.tick(now, message)
        self._user_callback(message)

    def __getattr__(self, name):
        return getattr(self._sub, name)

    def add_check(self, check):
        check.name = self._sub.name + ' subscriber ' + check.name
        self._checks.append(check)
        return check
