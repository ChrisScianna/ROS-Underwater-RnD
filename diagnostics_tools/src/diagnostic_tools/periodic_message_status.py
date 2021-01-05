import collections

from diagnostic_tools.periodic_event_status import PeriodicEventStatus
from diagnostic_tools.topic_diagnostic_task import TopicDiagnosticTask


class PeriodicMessageStatus(TopicDiagnosticTask):

    @staticmethod
    def configure(*args, **kwargs):
        return PeriodicEventStatus.configure(*args, **kwargs)

    def __init__(self, name='rate check', config=None, **kwargs):
        TopicDiagnosticTask.__init__(self, name, **kwargs)
        self._impl = PeriodicEventStatus(name, config=config, **kwargs)

    def tick(self, message=None, time=None):
        return self._impl.tick(time=time)

    def run(self, stat):
        return self._impl.run(stat)
