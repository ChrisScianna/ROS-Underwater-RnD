import threading

import diagnostic_updater


class PeriodicDiagnosticTask(diagnostic_updater.DiagnosticTask):

    def __init__(self, name):
        diagnostic_updater.DiagnosticTask.__init__(self, name)
        self._lock = threading.Lock()

    def tick(self, time):
        raise NotImplementedError()
