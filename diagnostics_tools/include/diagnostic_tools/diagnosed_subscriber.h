/*
 * diagnostic_tools/diagnosed_subscriber.h
 */

#ifndef _DIAGNOSTIC_TOOLS__DIAGNOSED_SUBSCRIBER_H_
#define _DIAGNOSTIC_TOOLS__DIAGNOSED_SUBSCRIBER_H_

#include <memory>
#include <stdexcept>
#include <utility>

#include <boost/make_unique.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna {
namespace diagnostic_tools {

using CompositeDiagnosticTask = diagnostic_updater::CompositeDiagnosticTask;

template <typename MessageT>
class DiagnosedSubscriber {
 public:
  using CallbackT = boost::function<void(const boost::shared_ptr<MessageT const> &)>;

  DiagnosedSubscriber() = default;

  DiagnosedSubscriber(const ros::NodeHandle &nh, const std::string &topic_name, uint32_t queue_size,
                      CallbackT user_callback, ros::CallbackQueueInterface *callback_queue)
      : impl_(boost::make_shared<Implementation>(nh, topic_name, queue_size,
                                                 std::move(user_callback), callback_queue)) {}

  std::string getTopic() const {
    if (!impl_) {
      throw std::logic_error("subscriber not initialized");
    }
    return impl_->getTopic();
  }

  template <template <typename T> typename Check, typename... ArgsT>
  Check<MessageT> &add_check(ArgsT &&... args) {
    return this->add_check<Check<MessageT>>(std::forward<ArgsT>(args)...);
  }

  template <typename CheckT, typename... ArgsT>
  CheckT &add_check(const std::string &name, ArgsT &&... args) {
    if (!impl_) {
      throw std::logic_error("subscriber not initialized");
    }
    return static_cast<CheckT *>(impl_->add_check(boost::make_unique<CheckT>(
        getTopic() + " subscriber " + name, std::forward<ArgsT>(args)...)));
  }

 private:
  class Implementation {
   public:
    Implementation(const ros::NodeHandle &nh, const std::string &topic_name, uint32_t queue_size,
                   CallbackT user_callback, ros::CallbackQueueInterface *callback_queue)
        : task_(topic_name + " subscription status"), user_callback_(std::move(user_callback)) {
      sub_ = nh.subscribe(topic_name, queue_size, &Implementation::callback, this, callback_queue);
    }

    TopicDiagnosticTask<MessageT> &add_check(std::unique_ptr<TopicDiagnosticTask<MessageT>> task) {
      checks_.push_back(std::move(task));
      return *checks_.back().get();
    }

    std::string getTopic() const { return sub_.getTopic(); }

   private:
    void callback(const typename MessageT::ConstPtr &message) {
      auto now = ros::Time::now();
      for (const auto &task : checks_) {
        task->tick(now, message);
      }
      user_callback_(message);
    }

    std::vector<std::unique_ptr<TopicDiagnosticTask<MessageT>>> checks_;
    CallbackT user_callback_;
    ros::Subscriber sub_;
  };

  boost::shared_ptr<Implementation> impl_;
};

template <typename MessageT, typename ClassT>
DiagnosedPublisher<MessageT> create_subscriber(
    const ros::NodeHandle &nh, const std::string &topic_name, uint32_t queue_size,
    void (ClassT::*method)(const boost::shared_ptr<MessageT const> &), ClassT *instance,
    ros::CallbackQueueInterface *callback_queue = nullptr) {
  return DiagnosedSubscriber<MessageT>(nh, topic_name, queue_size,
                                       boost::bind(method, instance, _1), callback_queue);
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // _DIAGNOSTIC_TOOLS__DIAGNOSED_SUBSCRIBER_H_
