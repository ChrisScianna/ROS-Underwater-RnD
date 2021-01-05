import itertools
import collections

from diagnostic_tools.diagnostic import Diagnostic
from diagnostic_tools.topic_diagnostic_task import TopicDiagnosticTask


class MessageStagnationCheck(TopicDiagnosticTask):

    Diagnostics = collections.namedtuple('Diagnostics', 'normal stagnation stale')
    Configuration = collections.namedtuple('Configuration', 'window_size diagnostics')

    @staticmethod
    def configure(
        window_size=3,
        normal_diagnostic=Diagnostic.OK,
        stagnation_diagnostic=Diagnostic.WARN,
        stale_diagnostic=Diagnostic.STALE
    ):
        normal_diagnostic = Diagnostic.build(normal_diagnostic)
        if not normal_diagnostic.description:
            normal_diagnostic.description = 'Messages fluctuate normally'
        stagnation_diagnostic = Diagnostic.build(stagnation_diagnostic)
        if not stagnation_diagnostic.description:
            stagnation_diagnostic.description = 'Messages have stagnated'
        stale_diagnostic = Diagnostic.build(stale_diagnostic)
        if not stale_diagnostic.description:
            stale_diagnostic.description = 'Not enough messages since last update'
        return MessageStagnationCheck.Configuration(
            window_size=window_size,
            diagnostics=MessageStagnationCheck.Diagnostics(
                normal=normal_diagnostic,
                stagnation=stagnation_diagnostic,
                stale=stale_diagnostic
            )
        )

    def __init__(self, equal_op, name='stagnation check', config=None, **kwargs):
        TopicDiagnosticTask.__init__(self, name, **kwargs)
        self._equal_op = equal_op
        if config is None:
            config = MessageStagnationCheck.configure()
        self._config = config

        self._message_window = collections.deque(maxlen=self._config.window_size)

    def tick(self, message=None, time=None):
        if not message:
            rospy.logwarn('No stagnation checks possible on null message')
            return
        with self._lock:
            self._message_window.append(message)

    def run(self, stat):
        with self._lock:
            if len(self._message_window) < self._config.window_size:
                diagnostic = self._config.diagnostics.stale
            else:
                head = self._message_window[0]
                tail = itertools.islice(self._message_window, 1, None)
                if all(self._equal_op(head, element) for element in tail):
                    diagnostic = self._config.diagnostics.stagnation
                else:
                    diagnostic = self._config.diagnostics.normal
            stat.summary(diagnostic.status, diagnostic.description)
            if diagnostic.code is not None:
                stat.add('Code', diagnostic.code)
            return stat
