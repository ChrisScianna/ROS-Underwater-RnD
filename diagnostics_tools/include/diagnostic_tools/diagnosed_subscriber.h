/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * diagnostic_tools/diagnosed_subscriber.h
 */

#ifndef DIAGNOSTIC_TOOLS_DIAGNOSED_SUBSCRIBER_H
#define DIAGNOSTIC_TOOLS_DIAGNOSED_SUBSCRIBER_H

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/make_unique.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna
{
namespace diagnostic_tools
{

using CompositeDiagnosticTask = diagnostic_updater::CompositeDiagnosticTask;

template <typename MessageT>
class DiagnosedSubscriber
{
public:
  using CallbackT = boost::function<void(const boost::shared_ptr<MessageT const> &)>;

  DiagnosedSubscriber() = default;

  DiagnosedSubscriber(const ros::NodeHandle &nh, const std::string &topic_name, uint32_t queue_size,
                      CallbackT user_callback, ros::CallbackQueueInterface *callback_queue)
    : impl_(boost::make_shared<Implementation>(nh, topic_name, queue_size,
            std::move(user_callback), callback_queue)) {}

  std::string getTopic() const
  {
    if (!impl_)
    {
      throw std::logic_error("subscriber not initialized");
    }
    return impl_->getTopic();
  }

  template <template <typename T> typename Check, typename... ArgsT>
  Check<MessageT> &add_check(ArgsT &&... args)  // NOLINT(build/c++11)
  {
    return this->add_check<Check<MessageT>>(std::forward<ArgsT>(args)...);
  }

  template <typename CheckT, typename... ArgsT>
  CheckT &add_check(const std::string &name, ArgsT &&... args)  // NOLINT(build/c++11)
  {
    if (!impl_)
    {
      throw std::logic_error("subscriber not initialized");
    }
    return static_cast<CheckT *>(impl_->add_check(boost::make_unique<CheckT>(
                                   getTopic() + " subscriber " + name, std::forward<ArgsT>(args)...)));
  }

private:
  class Implementation
  {
  public:
    Implementation(const ros::NodeHandle &nh, const std::string &topic_name, uint32_t queue_size,
                   CallbackT user_callback, ros::CallbackQueueInterface *callback_queue)
      : task_(topic_name + " subscription status"), user_callback_(std::move(user_callback))
    {
      sub_ = nh.subscribe(topic_name, queue_size, &Implementation::callback, this, callback_queue);
    }

    TopicDiagnosticTask<MessageT> &add_check(std::unique_ptr<TopicDiagnosticTask<MessageT>> task)
    {
      checks_.push_back(std::move(task));
      return *checks_.back().get();
    }

    std::string getTopic() const
    {
      return sub_.getTopic();
    }

  private:
    void callback(const typename MessageT::ConstPtr &message)
    {
      for (const auto &task : checks_)
      {
        task->tick(message);
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
  ros::CallbackQueueInterface *callback_queue = nullptr)
{
  return DiagnosedSubscriber<MessageT>(nh, topic_name, queue_size,
                                       boost::bind(method, instance, _1), callback_queue);
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_DIAGNOSED_SUBSCRIBER_H
