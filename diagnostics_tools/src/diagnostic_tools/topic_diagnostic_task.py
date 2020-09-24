from diagnostic_tools.periodic_diagnostic_task import PeriodicDiagnosticTask


class TopicDiagnosticTask(PeriodicDiagnosticTask):

    def tick(self, time, message=None):
        raise NotImplementedError()
