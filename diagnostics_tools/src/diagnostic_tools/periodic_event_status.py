import collections

import rospy

from diagnostic_tools.diagnostic import Diagnostic
from diagnostic_tools.periodic_diagnostic_task import PeriodicDiagnosticTask
from diagnostic_tools.sampled_statistics import SampledStatistics


class PeriodicEventStatus(PeriodicDiagnosticTask):

    Diagnostics = collections.namedtuple('Diagnostics', 'normal abnormal stale')
    Configuration = collections.namedtuple(
        'Configuration', 'min_acceptable_period max_acceptable_period diagnostics'
    )

    @staticmethod
    def configure(
        min_acceptable_period=-1,
        max_acceptable_period=5,
        normal_diagnostic=Diagnostic.OK,
        abnormal_diagnostic=Diagnostic.WARN,
        stale_diagnostic=Diagnostic.STALE
    ):
        normal_diagnostic = Diagnostic.build(normal_diagnostic)
        if not normal_diagnostic.description:
            normal_diagnostic.description = 'Rate within tolerance'
        abnormal_diagnostic = Diagnostic.build(abnormal_diagnostic)
        if not abnormal_diagnostic.description:
            abnormal_diagnostic.description = 'Rate too high or too low'
        stale_diagnostic = Diagnostic.build(stale_diagnostic)
        if not stale_diagnostic.description:
            stale_diagnostic.description = 'Not enough data since last update'
        return PeriodicEventStatus.Configuration(
            min_acceptable_period=min_acceptable_period,
            max_acceptable_period=max_acceptable_period,
            diagnostics=PeriodicEventStatus.Diagnostics(
                normal=normal_diagnostic,
                abnormal=abnormal_diagnostic,
                stale=stale_diagnostic
            )
        )

    def __init__(self, name='rate check', config=None, **kwargs):
        PeriodicDiagnosticTask.__init__(self, name, **kwargs)
        if config is None:
            config = PeriodicEventStatus.configure()
        self._config = config

        self._last_time = rospy.Time.now()
        self._historic_period = SampledStatistics(float)
        self._last_cycle_period = SampledStatistics(float)

    def tick(self, time):
        with self._lock:
            if self._last_time.is_zero():
                rospy.logwarn(
                    'Time went backwards from %f to %f, resetting.',
                    self._last_time.to_sec(), time.to_sec()
                )
                self._last_cycle_period.reset()
            else:
                delta_in_seconds = (time - self._last_time).to_sec()
                self._last_cycle_period.update(delta_in_seconds)
                self._historic_period.update(delta_in_seconds)
            self._last_time = time

    def run(self, stat):
        with self._lock:
            if self._last_cycle_period.sample_count > 0:
                diagnostic = self._config.diagnostics.normal
                if self._last_cycle_period.average < self._config.min_acceptable_period or \
                   self._last_cycle_period.average > self._config.max_acceptable_period:
                    diagnostic = self._config.diagnostics.abnormal
                stat.summary(diagnostic.status, diagnostic.description)
                stat.add(
                    'Average period (last cycle)', self._last_cycle_period.average)
                stat.add(
                    'Minimum period (last cycle)', self._last_cycle_period.minimum)
                stat.add(
                    'Maximum period (last cycle)', self._last_cycle_period.maximum)
            else:
                stat.summary(self._config.diagnostics.stale.status,
                             self._config.diagnostics.stale.description)
            if self._historic_period.sample_count > 0:
                stat.add(
                    'Average period (historic)', self._historic_period.average)
                stat.add(
                    'Minimum period (historic)', self._historic_period.minimum)
                stat.add(
                    'Maximum period (historic)', self._historic_period.maximum)

            stat.add('Minimum acceptable period', self._config.min_acceptable_period)
            stat.add('Maximum acceptable period', self._config.max_acceptable_period)

            self._last_cycle_period.reset()
        return stat
