"""
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
"""

import collections

import rospy

from diagnostic_tools.diagnostic import Diagnostic
from diagnostic_tools.periodic_diagnostic_task import PeriodicDiagnosticTask
from diagnostic_tools.sampled_statistics import SampledStatistics


class PeriodicEventStatus(PeriodicDiagnosticTask):

    Diagnostics = collections.namedtuple('Diagnostics', 'normal abnormal stale')
    Configuration = collections.namedtuple(
        'Configuration',
        ' '.join([
            'min_acceptable_period max_acceptable_period max_reasonable_period',
            'short_term_avg_window long_term_avg_window diagnostics',
        ])
    )

    @staticmethod
    def configure(
        min_acceptable_period=-1,
        max_acceptable_period=5,
        max_reasonable_period=None,
        short_term_avg_window=1,
        long_term_avg_window=10000,
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
        if max_reasonable_period is None:
            max_reasonable_period = 5 * max_acceptable_period
        return PeriodicEventStatus.Configuration(
            min_acceptable_period=min_acceptable_period,
            max_acceptable_period=max_acceptable_period,
            max_reasonable_period=max_reasonable_period,
            short_term_avg_window=short_term_avg_window,
            long_term_avg_window=long_term_avg_window,
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

        self._last_time = rospy.Time(0)
        self._short_term_period = SampledStatistics(
            self._config.short_term_avg_window, dtype=float
        )
        self._long_term_period = SampledStatistics(
            self._config.long_term_avg_window, dtype=float
        )

    def tick(self, time=None):
        time = time or rospy.Time.now()
        with self._lock:
            if not self._last_time.is_zero():
                if self._last_time <= time:
                    delta_in_seconds = (time - self._last_time).to_sec()
                    if delta_in_seconds <= self._config.max_reasonable_period:
                        self._short_term_period.update(delta_in_seconds)
                        self._long_term_period.update(delta_in_seconds)
                    else:
                        rospy.logdebug(
                            'Time delta %f too long, ignoring', delta_in_seconds
                        )
                else:
                    rospy.logdebug(
                        'Time went backwards from %f to %f, ignoring',
                        self._last_time.to_sec(), time.to_sec()
                    )
            self._last_time = time

    def run(self, stat):
        with self._lock:
            diagnostic = self._config.diagnostics.normal
            if rospy.Time.now() - self._last_time > self._config.max_reasonable_period:
                # Unreasonable time delta, event is likely stale.
                self._short_term_period.reset()
            if self._short_term_period.sample_count > 0:
                if self._last_cycle_period.average < self._config.min_acceptable_period or \
                   self._last_cycle_period.average > self._config.max_acceptable_period:
                    diagnostic = self._config.diagnostics.abnormal
                stat.summary(diagnostic.status, diagnostic.description)
                if diagnostic.code is not None:
                    stat.add('Code', diagnostic.code)
                stat.add(
                    'Average period (last cycle)', self._last_cycle_period.average)
                stat.add(
                    'Minimum period (last cycle)', self._last_cycle_period.minimum)
                stat.add(
                    'Maximum period (last cycle)', self._last_cycle_period.maximum)
            else:
                diagnostic = self._config.diagnostics.stale
                stat.summary(diagnostic.status, diagnostic.description)
                if diagnostic.code is not None:
                    stat.add('Code', diagnostic.code)
            if self._long_term_period.sample_count > 0:
                stat.add(
                    'Average period (historic)', self._long_term_period.average)
                stat.add(
                    'Minimum period (historic)', self._long_term_period.minimum)
                stat.add(
                    'Maximum period (historic)', self._long_term_period.maximum)
            stat.add('Minimum acceptable period', self._config.min_acceptable_period)
            stat.add('Maximum acceptable period', self._config.max_acceptable_period)
        return stat
