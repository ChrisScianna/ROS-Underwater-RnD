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
 * diagnostic_tools/diagnosed_publisher.h
 */

#ifndef DIAGNOSTIC_TOOLS_DIAGNOSED_PUBLISHER_H
#define DIAGNOSTIC_TOOLS_DIAGNOSED_PUBLISHER_H

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

template <typename MessageT>
class DiagnosedPublisher
{
public:
  explicit DiagnosedPublisher(const ros::Publisher &pub)
    : impl_(boost::make_shared<Implementation>(pub)) {}

  DiagnosedPublisher() = default;

  std::string getTopic() const
  {
    if (!impl_)
    {
      throw std::logic_error("publisher not initialized");
    }
    return impl_->getTopic();
  }

  void publish(const MessageT &message)
  {
    if (!impl_)
    {
      throw std::logic_error("publisher not initialized");
    }
    impl_->publish(message);
  }

  void publish(const boost::shared_ptr<MessageT const> &message)
  {
    if (!impl_)
    {
      throw std::logic_error("publisher not initialized");
    }
    impl_->publish(message);
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
      throw std::logic_error("publisher not initialized");
    }
    return static_cast<CheckT &>(impl_->add_check(boost::make_unique<CheckT>(
                                   getTopic() + " publisher " + name, std::forward<ArgsT>(args)...)));
  }

private:
  class Implementation
  {
  public:
    explicit Implementation(const ros::Publisher &pub) : pub_(pub) {}

    std::string getTopic() const
    {
      return pub_.getTopic();
    }

    void publish(const MessageT &message)
    {
      pub_.publish(message);
      auto now = ros::Time::now();
      for (const auto &task : checks_)
      {
        task->tick(now, message);
      }
    }

    void publish(const boost::shared_ptr<MessageT const> &message)
    {
      pub_.publish(message);
      for (const auto &task : checks_)
      {
        task->tick(message);
      }
    }

    TopicDiagnosticTask<MessageT> &add_check(std::unique_ptr<TopicDiagnosticTask<MessageT>> task)
    {
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
DiagnosedPublisher<MessageT> create_publisher(ros::NodeHandle &nh, ArgsT... args)
{
  return DiagnosedPublisher<MessageT>(nh.advertise<MessageT>(args...));
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_DIAGNOSED_PUBLISHER_H
