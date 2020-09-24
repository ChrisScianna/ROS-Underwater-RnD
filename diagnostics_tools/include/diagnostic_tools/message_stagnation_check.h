/*
 * diagnostic_tools/message_stagnation_check.h
 */

#ifndef _DIAGNOSTIC_TOOLS__MESSAGE_STAGNATION_CHECK_H_
#define _DIAGNOSTIC_TOOLS__MESSAGE_STAGNATION_CHECK_H_

#include <deque>
#include <mutex>
#include <string>
#include <utility>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/diagnostic.h"
#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna {
namespace diagnostic_tools {

class MessageStagnationCheckParams final {
 public:
  MessageStagnationCheckParams() = default;

  MessageStagnationCheckParams &window_size(size_t size) {
    window_size_ = size;
    return *this;
  }

  size_t window_size() const { return window_size_; }

  MessageStagnationCheckParams &stagnation_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Messages have stagnated");
    }
    stagnation_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stagnation_diagnostic() const { return stagnation_diagnostic_; }

  MessageStagnationCheckParams &stale_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Not enough messages since last update");
    }
    stale_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stale_diagnostic() const { return stale_diagnostic_; }

  MessageStagnationCheckParams &normal_diagnostic(Diagnostic diagnostic) {
    if (diagnostic.description().empty()) {
      diagnostic.description("Messages fluctuate normally");
    }
    normal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &normal_diagnostic() const { return normal_diagnostic_; }

 private:
  size_t window_size_{3};
  Diagnostic normal_diagnostic_{Diagnostic::OK, "Messages fluctuate normally"};
  Diagnostic stagnation_diagnostic_{Diagnostic::WARN, "Messages have stagnated"};
  Diagnostic stale_diagnostic_{Diagnostic::STALE, "Not enough messages since last update"};
};

template <typename MessageT>
class MessageStagnationCheck : public TopicDiagnosticTask<MessageT> {
 public:
  using MessageEqualOp = boost::function<bool(const MessageT &, const MessageT &)>;

  MessageStagnationCheck(const std::string &name, const MessageEqualOp &equal_op)
      : MessageStagnationCheck(name, equal_op, MessageStagnationCheckParams{}) {}

  MessageStagnationCheck(const std::string &name, const MessageEqualOp &equal_op,
                         MessageStagnationCheckParams params)
      : TopicDiagnosticTask<MessageT>(name), equal_op_(equal_op), params_(std::move(params)) {}

  void tick(const ros::Time &stamp, const MessageT &message) override {
    tick(stamp, boost::make_shared<MessageT const>(message));
  }

  void tick(const ros::Time &stamp, const boost::shared_ptr<MessageT const> &message) override {
    if (!message) {
      ROS_WARN_NAMED("diagnostic_tools", "No stagnation checks possible on null message");
      return;
    }

    std::lock_guard<std::mutex> guard(mutex_);
    message_window_.push_back(message);
    if (message_window_.size() > params_.window_size()) {
      message_window_.pop_front();
    }
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override {
    std::lock_guard<std::mutex> guard(mutex_);
    const Diagnostic &diagnostic = diagnose();
    stat.summary(diagnostic.status(), diagnostic.description());
    if (diagnostic.has_code()) {
      stat.add("Code", diagnostic.code());
    }
  }

 private:
  const Diagnostic &diagnose() {
    if (message_window_.size() < params_.window_size()) {
      return params_.stale_diagnostic();
    }
    if (std::adjacent_find(message_window_.begin(), message_window_.end(),
                           [this](const boost::shared_ptr<MessageT const> &a,
                                  const boost::shared_ptr<MessageT const> &b) {
                             return equal_op_(*a, *b);
                           }) != message_window_.end()) {
      return params_.stagnation_diagnostic();
    }
    return params_.normal_diagnostic();
  }

  MessageEqualOp equal_op_;
  MessageStagnationCheckParams params_;
  std::deque<boost::shared_ptr<MessageT const>> message_window_;
  std::mutex mutex_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__MESSAGE_STAGNATION_CHECK_H_
