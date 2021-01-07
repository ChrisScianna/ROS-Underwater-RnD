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

import threading

import diagnostic_updater

from diagnostic_tools.diagnostic import Diagnostic


class HealthCheck(diagnostic_updater.DiagnosticTask):

    _default_descriptions = {
        Diagnostic.OK: 'Everything OK',
        Diagnostic.WARN: "Something doesn't seem right",
        Diagnostic.ERROR: "Something's wrong",
        Diagnostic.STALE: 'Not enough tests conducted'
    }

    def __init__(self, name, test_fn):
        diagnostic_updater.DiagnosticTask.__init__(self, name)
        self._test_fn = test_fn
        self._lock = threading.Lock()

    def test(self, *args, **kwargs):
        with self._lock:
            self._diagnostic = Diagnostic.build(self._test_fn(*args, **kwargs))
            if self._diagnostic.description is None:
                self._diagnostic.description = \
                    HealthCheck._default_descriptions[
                        self._diagnostic.status
                    ]
            return self._diagnostic.status == Diagnostic.OK

    def run(self, stat):
        with self._lock:
            stat.summary(self._diagnostic.status, self._diagnostic.description)
            if self._diagnostic.code is not None:
                stat.add("Code", self._diagnostic.code)
            return stat
