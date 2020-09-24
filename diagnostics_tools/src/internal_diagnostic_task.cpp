#include "diagnostic_tools/internal_diagnostic_task.h"

#include <cstdarg>
#include <memory>

namespace qna {
namespace diagnostic_tools {

using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;

InternalDiagnosticTask::InternalDiagnosticTask(const std::string &name)
    : InternalDiagnosticTask(name, false) {}

InternalDiagnosticTask::InternalDiagnosticTask(const std::string &name, bool critical)
    : diagnostic_updater::DiagnosticTask(name), critical_(critical) {}

bool InternalDiagnosticTask::check(const std::string &name, bool condition) {
  return check(name, condition, "NOT OK");
}

bool InternalDiagnosticTask::check(const std::string &name, bool condition, const char *format,
                                   ...) {
  va_list va;
  va_start(va, format);
  try {
    std::lock_guard<std::mutex> guard(mutex_);
    status_[name].ok &= condition;
    if (!condition) {
      va_list other_va;
      va_copy(other_va, va);
      int size = vsnprintf(NULL, 0, format, other_va);
      if (size < 0) {
        throw std::runtime_error("Bad format");
      }
      std::unique_ptr<char> buffer(new char[size + 1]);
      if (vsnprintf(buffer.get(), size, format, va) < 0) {
        throw std::runtime_error("Bad format");
      }
      status_[name].error = buffer.get();
    }
  } catch (...) {
    va_end(va);
    throw;
  }
  return condition;
}

void InternalDiagnosticTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  std::lock_guard<std::mutex> guard(mutex_);
  bool ok = true;
  for (const auto &kv : status_) {
    ok &= kv.second.ok;
    stat.add(kv.first, kv.second.ok ? "OK" : kv.second.error);
  }
  if (!ok) {
    auto level = critical_ ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN;
    stat.summary(level, "Some checks failed");
  } else {
    stat.summary(DiagnosticStatus::OK, "Everything OK");
  }
  status_.clear();
}

}  // namespace diagnostic_tools
}  // namespace qna
