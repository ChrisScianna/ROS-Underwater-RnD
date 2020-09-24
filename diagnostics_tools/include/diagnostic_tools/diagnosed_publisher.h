/*
 * diagnostic_tools/diagnosed_publisher.h
 */

#ifndef _DIAGNOSTICS_TOOLS__DIAGNOSED_PUBLISHER_H_
#define _DIAGNOSTICS_TOOLS__DIAGNOSED_PUBLISHER_H_

#include <memory>
#include <stdexcept>
#include <utility>

#include <boost/make_unique.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna {
namespace diagnostic_tools {

template <typename MessageT>
class DiagnosedPublisher {
 public:
  DiagnosedPublisher(const ros::Publisher &pub) : impl_(boost::make_shared<Implementation>(pub)) {}

  DiagnosedPublisher() = default;

  std::string getTopic() const {
    if (!impl_) {
      throw std::logic_error("publisher not initialized");
    }
    return impl_->getTopic();
  }

  void publish(const MessageT &message) {
    if (!impl_) {
      throw std::logic_error("publisher not initialized");
    }
    impl_->publish(message);
  }

  void publish(const boost::shared_ptr<MessageT const> &message) {
    if (!impl_) {
      throw std::logic_error("publisher not initialized");
    }
    impl_->publish(message);
  }

  template <template <typename T> typename Check, typename... ArgsT>
  Check<MessageT> &add_check(ArgsT &&... args) {
    return this->add_check<Check<MessageT>>(std::forward<ArgsT>(args)...);
  }

  template <typename CheckT, typename... ArgsT>
  CheckT &add_check(const std::string &name, ArgsT &&... args) {
    if (!impl_) {
      throw std::logic_error("publisher not initialized");
    }
    return static_cast<CheckT &>(impl_->add_check(boost::make_unique<CheckT>(
        getTopic() + " publisher " + name, std::forward<ArgsT>(args)...)));
  }

 private:
  class Implementation {
   public:
    Implementation(const ros::Publisher &pub) : pub_(pub) {}

    std::string getTopic() const { return pub_.getTopic(); }

    void publish(const MessageT &message) {
      pub_.publish(message);
      auto now = ros::Time::now();
      for (const auto &task : checks_) {
        task->tick(now, message);
      }
    }

    void publish(const boost::shared_ptr<MessageT const> &message) {
      pub_.publish(message);
      auto now = ros::Time::now();
      for (const auto &task : checks_) {
        task->tick(now, message);
      }
    }

    TopicDiagnosticTask<MessageT> &add_check(std::unique_ptr<TopicDiagnosticTask<MessageT>> task) {
      checks_.push_back(std::move(task));
      return *checks_.back().get();
    }

   private:
    std::vector<std::unique_ptr<TopicDiagnosticTask<MessageT>>> checks_;
    ros::Publisher pub_;
  };

  boost::shared_ptr<Implementation> impl_;
};

template <typename MessageT, typename... ArgsT>
DiagnosedPublisher<MessageT> create_publisher(ros::NodeHandle &nh, ArgsT... args) {
  return DiagnosedPublisher<MessageT>(nh.advertise<MessageT>(args...));
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTICS_TOOLS__DIAGNOSED_PUBLISHER_H_
