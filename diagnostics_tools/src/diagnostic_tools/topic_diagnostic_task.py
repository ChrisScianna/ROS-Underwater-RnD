from diagnostic_tools.periodic_diagnostic_task import PeriodicDiagnosticTask


class TopicDiagnosticTask(PeriodicDiagnosticTask):

    def tick(self, message=None, time=None):
        raise NotImplementedError()
