/*
 * diagnostic_tools/internal_diagnostic_task.h
 */

#ifndef _DIAGNOSTIC_TOOLS__INTERNAL_DIAGNOSTIC_TASK_H_
#define _DIAGNOSTIC_TOOLS__INTERNAL_DIAGNOSTIC_TASK_H_

#include <map>
#include <mutex>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace qna {
namespace diagnostic_tools {

class InternalDiagnosticTask : public diagnostic_updater::DiagnosticTask {
 public:
  InternalDiagnosticTask(const std::string &name);
  InternalDiagnosticTask(const std::string &name, bool critical);

  bool check(const std::string &name, bool condition);
  bool check(const std::string &name, bool condition, const char *format, ...);

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

 private:
  struct check_t {
    bool ok{true};
    std::string error;
  };
  std::map<std::string, check_t> status_;
  std::mutex mutex_;
  bool critical_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__INTERNAL_DIAGNOSTIC_TASK_H_
