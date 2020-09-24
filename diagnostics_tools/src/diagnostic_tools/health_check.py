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
