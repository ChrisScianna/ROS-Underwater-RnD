/*
 * diagnostic_tools/topic_diagnostic_task.h
 */

#ifndef _DIAGNOSTIC_TOOLS__TOPIC_DIAGNOSTIC_TASK_H_
#define _DIAGNOSTIC_TOOLS__TOPIC_DIAGNOSTIC_TASK_H_

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/periodic_diagnostic_task.h"

namespace qna {
namespace diagnostic_tools {

template <typename MessageT>
class TopicDiagnosticTask : public PeriodicDiagnosticTask {
 public:
  using PeriodicDiagnosticTask::PeriodicDiagnosticTask;

  void tick(const MessageT& message) {
    tick(ros::Time::now(), message);
  }

  void tick(const boost::shared_ptr<MessageT const>& message) {
    tick(ros::Time::now(), message);
  }

  virtual void tick(const ros::Time& stamp, const MessageT& message) = 0;

  virtual void tick(const ros::Time& stamp, const boost::shared_ptr<MessageT const>& message) = 0;

  void tick(const ros::Time& stamp) override { this->tick(stamp, nullptr); }
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTICS_TOOLS
