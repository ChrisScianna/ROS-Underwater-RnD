/*
 * diagnostic_tools/periodic_diagnostic_task.h
 */

#ifndef _DIAGNOSTIC_TOOLS__PERIODIC_DIAGNOSTIC_TASK_H_
#define _DIAGNOSTIC_TOOLS__PERIODIC_DIAGNOSTIC_TASK_H_

#include <diagnostic_updater/diagnostic_updater.h>

namespace qna {
namespace diagnostic_tools {

class PeriodicDiagnosticTask : public diagnostic_updater::DiagnosticTask {
 public:
  using diagnostic_updater::DiagnosticTask::DiagnosticTask;

  void tick() { tick(ros::Time::now()); }

  virtual void tick(const ros::Time& t) = 0;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__PERIODIC_DIAGNOSTIC_TASK_H_
