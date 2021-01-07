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
