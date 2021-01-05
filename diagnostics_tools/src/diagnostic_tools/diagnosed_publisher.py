import rospy


class DiagnosedPublisher:

    def __init__(self, *args, **kwargs):
        self._pub = rospy.Publisher(*args, **kwargs)
        self._checks = []

    def __getattr__(self, name):
        return getattr(self._pub, name)

    def publish(self, message):
        self._pub.publish(message)
        for task in self._checks:
            task.tick(message)

    def add_check(self, check):
        check.name = self._pub.name + ' publisher ' + check.name
        self._checks.append(check)
        return check
