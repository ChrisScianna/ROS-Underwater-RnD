/*
 * diagnostic_tools/periodic_message_status.h
 */

#ifndef _DIAGNOSTIC_TOOLS__PERIODIC_MESSAGE_STATUS_H_
#define _DIAGNOSTIC_TOOLS__PERIODIC_MESSAGE_STATUS_H_

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/periodic_event_status.h"
#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna {
namespace diagnostic_tools {

using PeriodicMessageStatusParams = PeriodicEventStatusParams;

template <typename MessageT>
class PeriodicMessageStatus : public TopicDiagnosticTask<MessageT> {
 public:
  PeriodicMessageStatus(const std::string &name)
      : PeriodicMessageStatus(name, PeriodicMessageStatusParams{}) {}

  PeriodicMessageStatus(const std::string &name, PeriodicMessageStatusParams params)
      : TopicDiagnosticTask<MessageT>(name), impl_(name, std::move(params)) {}

  void tick(const ros::Time &stamp, const MessageT &) override { this->tick(stamp); }

  void tick(const ros::Time &stamp, const boost::shared_ptr<MessageT const> &) override {
    this->tick(stamp);
  }

  void tick(const ros::Time &stamp) override { impl_.tick(stamp); }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override { impl_.run(stat); }

 private:
  PeriodicEventStatus impl_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__PERIODIC_MESSAGE_STATUS_H_
