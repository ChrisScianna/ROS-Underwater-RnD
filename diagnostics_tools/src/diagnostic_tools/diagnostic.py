
from diagnostic_msgs.msg import DiagnosticStatus


class Diagnostic:
    OK = DiagnosticStatus.OK
    WARN = DiagnosticStatus.WARN
    ERROR = DiagnosticStatus.ERROR
    STALE = DiagnosticStatus.STALE

    @classmethod
    def build(cls, proto):
        if isinstance(proto, Diagnostic):
            return proto
        if not isinstance(proto, tuple):
            proto = (proto,)
        return cls(*proto)

    def __init__(self, status=OK, description=None, code=None):
        self.status = status
        self.description = description
        self.code = code
